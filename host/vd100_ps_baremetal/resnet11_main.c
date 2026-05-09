/* =============================================================================
 * resnet11_main.c — VD100 板 baremetal lwIP server
 *
 * 跑在 Versal A72 baremetal (无 Linux), 用 Xilinx lwIP TCP stack 收 PC 发的图
 * 推理后回传. 跟 host/vd100_ps/resnet11_server.c (Linux 版) 协议一致.
 *
 * Vitis 创建工程时选 "lwIP Echo Server" 模板, 把 src/echo.c 替换成本文件 (改名
 * resnet11_main.c). main.c 调 start_application() 时调本文件的 start_resnet11_server.
 *
 * 协议 (跟 host/vd100_pc/resnet11_client.py 配对):
 *   REQ:  magic 'FXNN' + cmd 1 + ifm_len + ifm bytes (2,073,600)
 *   RESP: magic 'FXNN' + status + cy + ofm_len + ofm bytes (522)
 *
 * 编译需要 Vitis 平台组件:
 *   - standalone domain
 *   - lwIP 2.1.x (echo server template 自带)
 *   - xilffs / xilrsa 不需要
 * ============================================================================= */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"

/* ----- 协议常量 ----- */
#define MAGIC      0x46584E4Eu
#define CMD_INFER  1u

/* ----- ResNet11 维度 ----- */
#define IFM_H        960
#define IFM_W        540
#define IFM_CIN      4
#define IFM_LEN      (IFM_H * IFM_W * IFM_CIN)   /* ~2 MB */
#define OFM_LEN      522

/* ----- 板级地址 (跟 BD address editor 一致) ----- */
#define DDR_BASE         0x00000000u    /* PS DDR4 low (PL m_axi 看到的地址) */
#define DDR_IFM_OFFSET   0x00000000u
#define DDR_DESC_OFFSET  0x00800000u    /* 8 MB 后 */
#define DDR_OFM_OFFSET   0x07F00000u    /* 127 MB 处 */

#define CSR_BASE         0xA0000000u    /* PS GP aperture */
#define CSR_PER_CORE     0x1000u
#define CSR_CTRL         0x000
#define CSR_STATUS       0x004
#define CSR_DESC_BASE    0x008
#define CSR_DESC_COUNT   0x00C
#define CSR_DMA_MODE     0x010

/* ----- 接收 state machine ----- */
typedef enum {
    ST_HEADER,    /* 等 12 byte req header */
    ST_IFM,       /* 收 IFM body */
    ST_INFER,     /* 已收完 IFM, 触发推理 */
    ST_SEND_RESP, /* 发 16 byte resp + OFM */
} rx_state_t;

typedef struct {
    rx_state_t state;
    uint8_t  hdr_buf[12];
    uint32_t hdr_got;
    uint32_t ifm_remaining;
    uint32_t ifm_offset;
    uint32_t cy_result;
} session_t;

/* ----- HW IO helpers ----- */
static inline void csr_write(int core, uint32_t off, uint32_t val) {
    Xil_Out32(CSR_BASE + core * CSR_PER_CORE + off, val);
}
static inline uint32_t csr_read(int core, uint32_t off) {
    return Xil_In32(CSR_BASE + core * CSR_PER_CORE + off);
}

/* ----- 推理 (blocking, ~2 ms) -----
 * IFM 已经写到 DDR_IFM_OFFSET (PC 端通过 lwIP recv → 我们 memcpy 到 DDR4).
 * 启动 3 ConvCore 同时跑, poll DONE.
 */
static uint32_t do_inference(void) {
    /* IFM 数据已经在 DDR4 IFM 区, cache flush 让 PL 能读到 */
    Xil_DCacheFlushRange(DDR_BASE + DDR_IFM_OFFSET, IFM_LEN);

    /* 配 3 个 ConvCore, 启动 */
    XTime t0, t1;
    XTime_GetTime(&t0);
    for (int c = 0; c < 3; c++) {
        csr_write(c, CSR_DESC_BASE,  DDR_BASE + DDR_DESC_OFFSET + c * 0x10000);
        csr_write(c, CSR_DESC_COUNT, 63);     /* ResNet11 desc 数 */
        csr_write(c, CSR_DMA_MODE,   3);
        csr_write(c, CSR_CTRL,       0x1);    /* START_DFE */
    }

    /* Poll done (实际生产用 IRQ, 这里 demo busy poll) */
    int done_mask = 0;
    while (done_mask != 0x7) {
        for (int c = 0; c < 3; c++) {
            if (!(done_mask & (1<<c)) && (csr_read(c, CSR_STATUS) & 0x10))
                done_mask |= (1<<c);
        }
    }
    XTime_GetTime(&t1);

    /* OFM 已写到 DDR_OFM_OFFSET, invalidate cache 让 A72 看到最新 */
    Xil_DCacheInvalidateRange(DDR_BASE + DDR_OFM_OFFSET, OFM_LEN);

    /* PS A72 1.5 GHz, XTime 是 32 KHz GTC, cycles = (t1-t0) * (1.5e9 / 32e3) */
    /* 简化: 返回 elapsed in COUNTS_PER_SECOND/100MHz 等价 PL cy */
    uint64_t elapsed_ns = ((t1 - t0) * 1000000000ull) / COUNTS_PER_SECOND;
    return (uint32_t)(elapsed_ns / 10);     /* 100 MHz: 10ns/cy */
}

/* ----- lwIP TCP recv callback ----- */
static err_t resnet11_recv(void *arg, struct tcp_pcb *tpcb,
                           struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        free(arg);
        return ERR_OK;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }

    session_t *s = (session_t *)arg;
    struct pbuf *q = p;
    uint16_t consumed = 0;

    while (q && consumed < p->tot_len) {
        uint8_t *data = (uint8_t *)q->payload;
        uint16_t len = q->len;
        uint16_t off = 0;

        while (off < len) {
            if (s->state == ST_HEADER) {
                uint32_t need = 12 - s->hdr_got;
                uint32_t have = len - off;
                uint32_t n = (need < have) ? need : have;
                memcpy(s->hdr_buf + s->hdr_got, data + off, n);
                s->hdr_got += n;
                off += n;
                if (s->hdr_got == 12) {
                    uint32_t magic   = ((uint32_t*)s->hdr_buf)[0];
                    uint32_t cmd     = ((uint32_t*)s->hdr_buf)[1];
                    uint32_t ifm_len = ((uint32_t*)s->hdr_buf)[2];
                    if (magic != MAGIC || cmd != CMD_INFER || ifm_len != IFM_LEN) {
                        xil_printf("[srv] bad header (magic=%08x cmd=%u len=%u)\r\n",
                                   magic, cmd, ifm_len);
                        tcp_close(tpcb);
                        pbuf_free(p);
                        free(s);
                        return ERR_VAL;
                    }
                    s->state = ST_IFM;
                    s->ifm_remaining = IFM_LEN;
                    s->ifm_offset = 0;
                }
            } else if (s->state == ST_IFM) {
                uint32_t have = len - off;
                uint32_t n = (have < s->ifm_remaining) ? have : s->ifm_remaining;
                memcpy((void *)(DDR_BASE + DDR_IFM_OFFSET + s->ifm_offset),
                       data + off, n);
                s->ifm_offset += n;
                s->ifm_remaining -= n;
                off += n;
                if (s->ifm_remaining == 0) {
                    s->state = ST_INFER;
                }
            }
            if (s->state == ST_INFER) break;
        }

        consumed += q->len;
        q = q->next;
    }

    /* ACK 收到的 byte */
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    /* 收齐 IFM → 推理 + 发 resp */
    if (s->state == ST_INFER) {
        xil_printf("[srv] IFM received, running inference...\r\n");
        s->cy_result = do_inference();
        xil_printf("[srv] inference done, cy=%u\r\n", s->cy_result);

        uint32_t resp_hdr[4] = {MAGIC, 0, s->cy_result, OFM_LEN};
        tcp_write(tpcb, resp_hdr, 16, TCP_WRITE_FLAG_COPY);
        tcp_write(tpcb, (void *)(DDR_BASE + DDR_OFM_OFFSET), OFM_LEN,
                  TCP_WRITE_FLAG_COPY);
        tcp_output(tpcb);

        /* reset state for next req on same connection */
        s->state = ST_HEADER;
        s->hdr_got = 0;
    }
    return ERR_OK;
}

static err_t resnet11_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    if (err != ERR_OK) return err;
    session_t *s = (session_t *)calloc(1, sizeof(session_t));
    s->state = ST_HEADER;
    tcp_arg(newpcb, s);
    tcp_recv(newpcb, resnet11_recv);
    xil_printf("[srv] new client connected\r\n");
    return ERR_OK;
}

/* lwIP echo template main 调这个 (替换 start_application) */
int start_resnet11_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        xil_printf("ERROR creating PCB\r\n");
        return -1;
    }
    err_t err = tcp_bind(pcb, IP_ADDR_ANY, 5000);
    if (err != ERR_OK) {
        xil_printf("ERROR tcp_bind: %d\r\n", err);
        return -2;
    }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, resnet11_accept);
    xil_printf("[srv] ResNet11 server listen on 5000\r\n");
    return 0;
}
