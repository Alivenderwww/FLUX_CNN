/* =============================================================================
 * resnet11_server.c — VD100 板上 PS A72 Linux socket server
 *
 * 跑在 ALINX VD100 板的 Petalinux 上. 通过 PS 千兆以太网 (eth0) 收 Windows PC 上位
 * 机发的 ResNet11 IFM, 配置 PL multicore_top_vd100 (3 ConvCore + Vitis NoC + DDR4)
 * 推理, 取 OFM 发回 PC.
 *
 * 数据通路:
 *   PC ──TCP──► A72 (本程序) ──mmap /dev/mem──► PS DDR4 (IFM, weight, desc)
 *                                                        ▲
 *                              PL ConvCore ──Versal NoC──┘
 *   PC ◄──TCP── A72 ◄──mmap ── PS DDR4 ◄── PL ODMA writes
 *
 * 编译 (Petalinux SDK 或 g++):
 *   aarch64-linux-gnu-gcc -O2 -o resnet11_server resnet11_server.c
 * 部署:
 *   scp resnet11_server root@<vd100_ip>:/home/root/
 *   ssh root@<vd100_ip>
 *   ./resnet11_server
 *
 * 跟 host/vd100_pc/resnet11_client.py 配对的协议 (见 client 注释).
 * ============================================================================= */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

/* ----- 协议常量 (跟 client 一致) ----- */
#define MAGIC      0x46584E4Eu  /* 'FXNN' */
#define CMD_INFER  1u

/* ----- ResNet11 参数 (跟 toolchain run_regression.py CASES 一致) ----- */
#define IFM_H        960
#define IFM_W        540
#define IFM_CIN      4
#define IFM_LEN      (IFM_H * IFM_W * IFM_CIN)   /* 2,073,600 byte ≈ 2 MB */
#define OFM_LEN      522                          /* FC 输出 logit 数 */

/* ----- DDR layout (PS DDR low 0x0_0000_0000 - 0x0_8000_0000 = 2GB) -----
 * 跟 toolchain/run_multicore_chain.py 的 DDR planner 输出对齐.
 * PL 通过 Versal NoC 访问 PS DDR, address 0x0_0000_0000 base.
 * PS A72 用 mmap /dev/mem 访问同一物理地址, 共享 IFM/OFM 数据. */
#define DDR_BASE_PHYS    0x00000000ul    /* 32-bit view, 跟 RTL 一致 */
#define DDR_IFM_OFFSET   0x00000000ul    /* IFM 区起点 */
#define DDR_DESC_OFFSET  0x00800000ul    /* desc list 起点 (8 MB 后) */
#define DDR_WEIGHT_OFFSET 0x00900000ul   /* weight 起点 */
#define DDR_OFM_OFFSET   0x07F00000ul    /* OFM 区 (7F MB 处, 留 1 MB) */
#define DDR_TOTAL_SIZE   0x08000000ul    /* 128 MB demo 用 (实际 PS DDR 2-4 GB) */

/* ----- PL CSR (经 PS GP AXI-Lite 接到 multicore_top_vd100.csr_axil) -----
 * Petalinux Device Tree 把这块映射成 /dev/uio0 (推荐) 或者直接 mmap /dev/mem.
 * 这里用 /dev/mem 简化 (UIO 需 DT 配置, 之后再换). */
#define CSR_BASE_PHYS    0xA4000000ul    /* PS M_AXI_FPD valid aperture (Versal default) */
#define CSR_PER_CORE     0x1000ul        /* 每 core 4KB CSR (csr_axil_addr[13:12] 选 core) */
#define CSR_REGIONLEN    0x4000ul        /* 总 16 KB (3 core × 4KB + spare) */

/* CSR offsets (跟 RTL/cfg_regs.sv 一致) */
#define CSR_CTRL         0x000  /* [0]=START_DFE, [1]=START_LAYER */
#define CSR_STATUS       0x004  /* [0]=DONE, [4]=DFE_DONE_STICKY */
#define CSR_DESC_BASE    0x008
#define CSR_DESC_COUNT   0x00C
#define CSR_DMA_MODE     0x010

/* ----- 全局 mmap 句柄 ----- */
static volatile uint32_t *csr_base[3] = {NULL, NULL, NULL};   /* 3 cores */
static volatile uint8_t  *ddr_base    = NULL;
static int memfd = -1;

/* ============================================================================
 * mmap 设置
 * ============================================================================ */
static int setup_mmap(void) {
    memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
        perror("open /dev/mem (need root?)");
        return -1;
    }
    /* DDR 数据区: 128 MB demo */
    ddr_base = mmap(NULL, DDR_TOTAL_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED,
                    memfd, DDR_BASE_PHYS);
    if (ddr_base == MAP_FAILED) {
        perror("mmap DDR");
        return -1;
    }
    /* PL CSR: 3 core × 4KB */
    void *csr = mmap(NULL, CSR_REGIONLEN, PROT_READ|PROT_WRITE, MAP_SHARED,
                     memfd, CSR_BASE_PHYS);
    if (csr == MAP_FAILED) {
        perror("mmap CSR");
        return -1;
    }
    for (int i = 0; i < 3; i++)
        csr_base[i] = (uint32_t *)((uint8_t *)csr + i * CSR_PER_CORE);
    fprintf(stderr, "[mmap] DDR @ %p, CSR @ %p (3 cores)\n", ddr_base, csr);
    return 0;
}

/* ============================================================================
 * 推理流程
 * ============================================================================ */
static uint32_t run_inference(const uint8_t *ifm, size_t ifm_len, uint8_t *ofm) {
    /* 1. IFM 写到 DDR IFM 区 */
    memcpy((void *)(ddr_base + DDR_IFM_OFFSET), ifm, ifm_len);
    __sync_synchronize();   /* mem barrier 保证 PL 看到 */

    /* 2. 配 3 个 ConvCore CSR (DESC_BASE + COUNT 由 driver 编译期写入 desc list) */
    /* 假设 desc list 已经预加载到 DDR_DESC_OFFSET (启动时一次性) */
    for (int c = 0; c < 3; c++) {
        csr_base[c][CSR_DESC_BASE/4]  = DDR_BASE_PHYS + DDR_DESC_OFFSET + c * 0x10000;
        csr_base[c][CSR_DESC_COUNT/4] = 63;   /* ResNet11 11 layer × ~6 desc/layer */
        csr_base[c][CSR_DMA_MODE/4]   = 3;    /* streaming row-ring */
    }

    /* 3. 启动 3 核 (并发 START_DFE) */
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int c = 0; c < 3; c++)
        csr_base[c][CSR_CTRL/4] = 0x1;        /* START_DFE */

    /* 4. Poll done (3 核全 done). 实际生产用 epoll /dev/uioN IRQ, 这里 demo 简化 poll */
    int done_mask = 0;
    while (done_mask != 0x7) {
        for (int c = 0; c < 3; c++) {
            if (!(done_mask & (1 << c)) && (csr_base[c][CSR_STATUS/4] & 0x10))
                done_mask |= (1 << c);
        }
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);

    /* 5. OFM 从 DDR 取 */
    memcpy(ofm, (void *)(ddr_base + DDR_OFM_OFFSET), OFM_LEN);

    /* 6. 计 cycles (估算 elapsed_ns × 100MHz, 实际生产从 PL hw counter 读) */
    uint64_t elapsed_ns = (t1.tv_sec - t0.tv_sec) * 1000000000ull
                        + (t1.tv_nsec - t0.tv_nsec);
    return (uint32_t)(elapsed_ns / 10);   /* 100 MHz: 10ns/cy */
}

/* ============================================================================
 * Socket server
 * ============================================================================ */
static int recv_all(int fd, void *buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = recv(fd, (uint8_t*)buf + got, len - got, 0);
        if (n <= 0) return -1;
        got += n;
    }
    return 0;
}

static int send_all(int fd, const void *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, (const uint8_t*)buf + sent, len - sent, 0);
        if (n <= 0) return -1;
        sent += n;
    }
    return 0;
}

static void handle_client(int cfd) {
    uint8_t *ifm = malloc(IFM_LEN);
    uint8_t ofm[OFM_LEN];
    if (!ifm) { close(cfd); return; }

    while (1) {
        /* 收 12 byte req header */
        uint32_t hdr[3];
        if (recv_all(cfd, hdr, 12) < 0) break;
        if (hdr[0] != MAGIC) {
            fprintf(stderr, "bad magic 0x%08x\n", hdr[0]);
            break;
        }
        if (hdr[1] != CMD_INFER) {
            fprintf(stderr, "bad cmd %u\n", hdr[1]);
            break;
        }
        if (hdr[2] != IFM_LEN) {
            fprintf(stderr, "bad ifm_len %u (expected %u)\n", hdr[2], IFM_LEN);
            break;
        }

        /* 收 IFM */
        if (recv_all(cfd, ifm, IFM_LEN) < 0) {
            fprintf(stderr, "recv ifm failed\n");
            break;
        }

        /* 推理 */
        uint32_t cy = run_inference(ifm, IFM_LEN, ofm);

        /* 发 16 byte resp header + OFM */
        uint32_t resp[4] = {MAGIC, 0 /*status*/, cy, OFM_LEN};
        if (send_all(cfd, resp, 16) < 0) break;
        if (send_all(cfd, ofm, OFM_LEN) < 0) break;
        fprintf(stderr, "[infer] cy=%u, ofm sent\n", cy);
    }

    free(ifm);
    close(cfd);
}

int main(int argc, char **argv) {
    int port = (argc > 1) ? atoi(argv[1]) : 5000;

    if (setup_mmap() < 0) return 1;

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in sa = {.sin_family = AF_INET,
                             .sin_addr.s_addr = INADDR_ANY,
                             .sin_port = htons(port)};
    if (bind(sfd, (struct sockaddr*)&sa, sizeof(sa)) < 0) {
        perror("bind"); return 1;
    }
    if (listen(sfd, 1) < 0) { perror("listen"); return 1; }

    fprintf(stderr, "[server] listen on 0.0.0.0:%d\n", port);
    while (1) {
        struct sockaddr_in ca;
        socklen_t cl = sizeof(ca);
        int cfd = accept(sfd, (struct sockaddr*)&ca, &cl);
        if (cfd < 0) { perror("accept"); continue; }
        fprintf(stderr, "[server] client %s:%d connected\n",
                inet_ntoa(ca.sin_addr), ntohs(ca.sin_port));
        handle_client(cfd);
    }
    return 0;
}
