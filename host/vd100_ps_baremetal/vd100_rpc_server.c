/* =============================================================================
 * vd100_rpc_server.c — VD100 板 baremetal lwIP RPC server (通用)
 *
 * 跑在 Versal A72 baremetal (无 Linux), 无业务逻辑. PC 通过 TCP RPC 推送任意数据
 * 到 DDR / 读 DDR / poke peek CSR / 触发 N 层推理. 板子烧 1 次 (bitstream + 这个
 * server), 模型/IFM/OFM 全部由 PC 网线动态发送.
 *
 * 跟 host/vd100_pc/vd100_rpc.py 配对. 协议跟仿真 (sim/tb_smc/tb_smc_chain.sv) 等价:
 *   sim TB host preload (hier ref 直写 mem) ↔  RPC LOAD_DDR (TCP 写 DDR)
 *   sim TB axi_lite_write (CSR)             ↔  RPC POKE_CSR
 *   sim TB write_dfe_start + wait_dfe_done  ↔  RPC RUN_LAYERS (a72 自跑层循环)
 *   sim TB read_mem_word (OFM check)         ↔  RPC READ_DDR
 *
 * Cache 一致性 (PL master 走 axi_noc S00..S02, **不**经过 A72 cache):
 *   PS 写 DDR 后 → DCacheFlushRange (push cache to DDR, PL 才能读到)
 *   PL 写 DDR 后 → A72 读前 DCacheInvalidateRange (扔 cache, 强迫从 DDR 读)
 *
 * 协议: 16-byte header + optional payload, 跟 lwIP TCP recv 流式对齐
 *   uint32 magic = 'FXNN' = 0x4E4E5846
 *   uint32 cmd
 *   uint32 arg0    (LOAD/READ: ddr_addr; POKE/PEEK: csr_offset; RUN: n_layers)
 *   uint32 arg1    (LOAD/READ: len; POKE: value; RUN: payload_len)
 *   <payload>      (LOAD: data; RUN: LayerCfg array; POKE/PEEK/PING: empty)
 *
 * Response: 16-byte header + optional payload
 *   uint32 magic
 *   uint32 status (0 = OK, !=0 = error)
 *   uint32 ret    (PEEK: csr value; RUN: 100MHz cycles; PING: 0xCAFEBABE)
 *   uint32 payload_len   (READ: data len; 其它 = 0)
 *   <payload>            (READ_DDR: data)
 *
 * Vitis 工程: 替换 lwIP echo server template 的 echo.c → 编进 ELF. main.c 调
 * start_application() 启动. 监听 TCP 5000.
 * ============================================================================= */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xil_mmu.h"     /* Xil_SetTlbAttributes for CSR aperture */
#include "xil_printf.h"
#include "xtime_l.h"

/* ========== 协议常量 ========== */
#define MAGIC           0x46584E4Eu   /* 'FXNN' little-endian */
#define CMD_LOAD_DDR    1u
#define CMD_READ_DDR    2u
#define CMD_POKE_CSR    3u
#define CMD_PEEK_CSR    4u
#define CMD_RUN_LAYERS  5u
#define CMD_PING        6u

#define STATUS_OK             0u
#define STATUS_BAD_MAGIC      1u
#define STATUS_BAD_CMD        2u
#define STATUS_BAD_LEN        3u
#define STATUS_RUN_TIMEOUT    4u
#define STATUS_OOM            5u

/* ========== 板级常量 ========== */
/* CSR aperture: BD address editor 实际给的是 0xA4000000-0xA4003FFF (16 KB).
 * pl_config.tcl source 写的 0xA0000000 但 Vivado 自动分配到 FPD AFI 默认 base 0xA4000000.
 * BD: csr_axil_awaddr[13:12] = core_id, [11:0] = core 内 reg offset.
 * Versal translation_table.S 标 0xA400_0000-0xAFFF_FFFF 是 FPD_AFI_0 Strongly Ordered (Device). */
#define CSR_BASE          0xA4000000u
#define CSR_PER_CORE      0x1000u

/* CSR register offset (跟 RTL/flux_cnn_params.svh 一致) */
#define CSR_CTRL          0x000u
#define CSR_STATUS        0x004u
#define CSR_DESC_LIST_BASE 0x180u
#define CSR_DESC_COUNT    0x184u

#define CTRL_START_DFE    0x10u   /* bit 4 */
#define STATUS_DONE       0x01u   /* bit 0 sticky (read-clear by start_dfe) */

#define NUM_CORES         3u
#define MAX_LAYERS        16u    /* ResNet11 N=11, 留余量 */

/* Versal A72 baremetal: PS DDR 物理地址 0x0_0000_0000 - 0x7_FFFFFFFF.
 * a72 ELF psv_ddr_low 占 0x0 - 0x01000000 (16 MB).
 * 用户 RPC LOAD_DDR 必须避开 [0, 0x01000000], 否则会破坏 ELF (我们不主动 reject,
 * 但默认 PC 端用 0x10000000 起 — 安全). */

/* ========== Layer cfg 结构 (跟 PC 端 vd100_rpc.py 一致) ========== */
/* 32 byte / layer, 双方按 little-endian 解释 */
typedef struct __attribute__((packed)) {
    uint32_t desc_list_base[NUM_CORES];   /* 0 = 该核本层不参与 */
    uint32_t desc_count   [NUM_CORES];
    uint32_t reserved     [2];            /* pad to 32 byte */
} layer_cfg_t;

/* ========== 接收 state machine ========== */
typedef enum {
    ST_HEADER,    /* 等 16 byte req header */
    ST_PAYLOAD,   /* 流式收 payload (LOAD_DDR / RUN_LAYERS) */
    ST_EXEC,      /* 收完 payload, 执行 cmd */
} rx_state_t;

typedef struct {
    rx_state_t state;
    uint8_t  hdr_buf[16];
    uint32_t hdr_got;
    /* current cmd context */
    uint32_t cmd;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t payload_remaining;
    uint32_t payload_offset;
    /* RUN_LAYERS payload 缓冲 (需要全收齐再跑) */
    uint8_t *run_payload;
} session_t;

/* ========== HW IO helpers ========== */
static inline void csr_write(uint32_t core, uint32_t off, uint32_t val) {
    Xil_Out32(CSR_BASE + core * CSR_PER_CORE + off, val);
}
static inline uint32_t csr_read(uint32_t core, uint32_t off) {
    return Xil_In32(CSR_BASE + core * CSR_PER_CORE + off);
}

/* ========== 跑 N 层推理 (sim TB tb_smc_chain.sv 等价) ========== */
static uint32_t run_layers(const layer_cfg_t *layers, uint32_t n_layers, uint32_t *out_cycles) {
    XTime t0, t1;
    XTime_GetTime(&t0);

    for (uint32_t l = 0; l < n_layers; l++) {
        const layer_cfg_t *cfg = &layers[l];
        uint32_t active_mask = 0;

        /* 串行写 boot regs (CSR 总线单口必须串行) + start_dfe */
        for (uint32_t c = 0; c < NUM_CORES; c++) {
            if (cfg->desc_count[c] == 0) continue;
            active_mask |= (1u << c);
            csr_write(c, CSR_DESC_LIST_BASE, cfg->desc_list_base[c]);
            csr_write(c, CSR_DESC_COUNT,     cfg->desc_count[c]);
            csr_write(c, CSR_CTRL,           CTRL_START_DFE);
        }

        if (active_mask == 0) continue;

        /* Poll done. 100 MHz, ResNet11 整网 ~2 ms = 200K cy.
         * 单层最坏 ~50K cy = 0.5 ms. 给 100 ms timeout 余量. */
        uint32_t done_mask = 0;
        uint32_t timeout_iters = 100000000u;  /* ~ 1 sec @ ~100 MHz poll */
        while (done_mask != active_mask && timeout_iters > 0) {
            for (uint32_t c = 0; c < NUM_CORES; c++) {
                if ((active_mask & (1u << c)) && !(done_mask & (1u << c))
                    && (csr_read(c, CSR_STATUS) & STATUS_DONE)) {
                    done_mask |= (1u << c);
                }
            }
            timeout_iters--;
        }
        if (timeout_iters == 0) {
            xil_printf("[srv] RUN layer %u TIMEOUT (active=0x%x done=0x%x)\r\n",
                       l, active_mask, done_mask);
            return STATUS_RUN_TIMEOUT;
        }
    }

    XTime_GetTime(&t1);
    /* Versal A72 timer = 100 MHz default (XPAR_CPU_CORTEXA72_0_TIMESTAMP_CLK_HZ).
     * 直接返回 timer ticks 当作 100 MHz cycles. */
    *out_cycles = (uint32_t)(t1 - t0);
    return STATUS_OK;
}

/* ========== 发 response header (16 byte) + 可选 payload ========== */
static err_t send_resp(struct tcp_pcb *tpcb, uint32_t status, uint32_t ret,
                       const void *payload, uint32_t payload_len) {
    uint32_t hdr[4] = {MAGIC, status, ret, payload_len};
    err_t err = tcp_write(tpcb, hdr, 16, TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) return err;
    if (payload && payload_len) {
        err = tcp_write(tpcb, payload, payload_len, TCP_WRITE_FLAG_COPY);
        if (err != ERR_OK) return err;
    }
    return tcp_output(tpcb);
}

/* ========== 执行 cmd (header 已收齐, 可能 payload 也已收齐) ========== */
static void exec_cmd(struct tcp_pcb *tpcb, session_t *s) {
    uint32_t status = STATUS_OK;
    uint32_t ret = 0;
    const void *resp_payload = NULL;
    uint32_t resp_len = 0;

    switch (s->cmd) {
        case CMD_LOAD_DDR:
            Xil_DCacheFlushRange((UINTPTR)s->arg0, s->arg1);
            break;

        case CMD_READ_DDR:
            Xil_DCacheInvalidateRange((UINTPTR)s->arg0, s->arg1);
            resp_payload = (const void *)(uintptr_t)s->arg0;
            resp_len     = s->arg1;
            break;

        case CMD_POKE_CSR: {
            /* arg0 = csr offset (含 [13:12] core_id), arg1 = value.
             * 直接写 CSR_BASE + arg0. */
            Xil_Out32(CSR_BASE + s->arg0, s->arg1);
            break;
        }

        case CMD_PEEK_CSR:
            ret = Xil_In32(CSR_BASE + s->arg0);
            break;

        case CMD_RUN_LAYERS: {
            uint32_t n_layers = s->arg0;
            uint32_t expected_len = n_layers * sizeof(layer_cfg_t);
            if (s->arg1 != expected_len || n_layers > MAX_LAYERS) {
                status = STATUS_BAD_LEN;
                xil_printf("[srv] RUN bad len: n_layers=%u arg1=%u expect=%u\r\n",
                           n_layers, s->arg1, expected_len);
                break;
            }
            uint32_t cy = 0;
            status = run_layers((const layer_cfg_t *)s->run_payload, n_layers, &cy);
            ret = cy;
            xil_printf("[srv] RUN %u layers done, cy=%u status=%u\r\n", n_layers, cy, status);
            break;
        }

        case CMD_PING:
            ret = 0xCAFEBABEu;
            break;

        default:
            status = STATUS_BAD_CMD;
            break;
    }

    send_resp(tpcb, status, ret, resp_payload, resp_len);

    /* 清理 RUN_LAYERS 临时缓冲 */
    if (s->run_payload) {
        free(s->run_payload);
        s->run_payload = NULL;
    }

    /* 重置 state, 收下一个请求 */
    s->state = ST_HEADER;
    s->hdr_got = 0;
}

/* ========== lwIP TCP recv callback ========== */
static err_t rpc_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        if (arg) {
            session_t *s = (session_t *)arg;
            if (s->run_payload) free(s->run_payload);
            free(s);
        }
        return ERR_OK;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }

    session_t *s = (session_t *)arg;
    struct pbuf *q = p;

    while (q && q->len > 0) {
        uint8_t *data = (uint8_t *)q->payload;
        uint16_t len = q->len;
        uint16_t off = 0;

        while (off < len) {
            if (s->state == ST_HEADER) {
                uint32_t need = 16 - s->hdr_got;
                uint32_t have = len - off;
                uint32_t n = (need < have) ? need : have;
                memcpy(s->hdr_buf + s->hdr_got, data + off, n);
                s->hdr_got += n;
                off += n;
                if (s->hdr_got == 16) {
                    /* 解 header */
                    uint32_t magic = ((uint32_t *)s->hdr_buf)[0];
                    s->cmd  = ((uint32_t *)s->hdr_buf)[1];
                    s->arg0 = ((uint32_t *)s->hdr_buf)[2];
                    s->arg1 = ((uint32_t *)s->hdr_buf)[3];
                    if (magic != MAGIC) {
                        xil_printf("[srv] bad magic 0x%08x\r\n", magic);
                        send_resp(tpcb, STATUS_BAD_MAGIC, 0, NULL, 0);
                        tcp_close(tpcb);
                        pbuf_free(p);
                        if (s->run_payload) free(s->run_payload);
                        free(s);
                        return ERR_VAL;
                    }

                    /* 决定是否需要收 payload */
                    if (s->cmd == CMD_LOAD_DDR) {
                        s->payload_remaining = s->arg1;
                        s->payload_offset    = 0;
                        s->state = (s->arg1 > 0) ? ST_PAYLOAD : ST_EXEC;
                    } else if (s->cmd == CMD_RUN_LAYERS) {
                        if (s->arg1 > MAX_LAYERS * sizeof(layer_cfg_t)) {
                            xil_printf("[srv] RUN payload too big %u\r\n", s->arg1);
                            send_resp(tpcb, STATUS_BAD_LEN, 0, NULL, 0);
                            s->state = ST_HEADER;
                            s->hdr_got = 0;
                            break;
                        }
                        s->run_payload = (uint8_t *)malloc(s->arg1);
                        if (!s->run_payload && s->arg1 > 0) {
                            send_resp(tpcb, STATUS_OOM, 0, NULL, 0);
                            s->state = ST_HEADER;
                            s->hdr_got = 0;
                            break;
                        }
                        s->payload_remaining = s->arg1;
                        s->payload_offset    = 0;
                        s->state = (s->arg1 > 0) ? ST_PAYLOAD : ST_EXEC;
                    } else {
                        /* POKE/PEEK/READ_DDR/PING: 无 payload, 直接执行 */
                        s->state = ST_EXEC;
                    }
                }
            } else if (s->state == ST_PAYLOAD) {
                uint32_t have = len - off;
                uint32_t n = (have < s->payload_remaining) ? have : s->payload_remaining;
                if (s->cmd == CMD_LOAD_DDR) {
                    /* 直接 memcpy 到 DDR (s->arg0 + offset) */
                    memcpy((void *)(uintptr_t)(s->arg0 + s->payload_offset),
                           data + off, n);
                } else if (s->cmd == CMD_RUN_LAYERS) {
                    /* 累积进 run_payload buffer */
                    memcpy(s->run_payload + s->payload_offset, data + off, n);
                }
                s->payload_offset    += n;
                s->payload_remaining -= n;
                off += n;
                if (s->payload_remaining == 0) s->state = ST_EXEC;
            }
            if (s->state == ST_EXEC) {
                /* 在 inner loop 里执行可能耗时长 (RUN_LAYERS 几 ms), lwIP 自动管 ack.
                 * 执行后 send_resp + 重置 state, 继续吃 buffer 里剩下的 byte. */
                exec_cmd(tpcb, s);
                /* exec_cmd 已 reset state ST_HEADER, hdr_got=0, 继续吃 */
            }
        }

        q = q->next;
        if (q == NULL) break;
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t rpc_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    if (err != ERR_OK) return err;
    session_t *s = (session_t *)calloc(1, sizeof(session_t));
    if (!s) return ERR_MEM;
    s->state = ST_HEADER;
    tcp_arg(newpcb, s);
    tcp_recv(newpcb, rpc_recv);
    /* 给 client 大 send window, RUN_LAYERS payload 多 ConvCore 大一点也容得下 */
    tcp_nagle_disable(newpcb);
    xil_printf("[srv] new client connected\r\n");
    return ERR_OK;
}

/* lwIP echo template: main.c 调 start_application() 启动 server */
int start_application(void) {
    /* D-cache enabled (BSP default, lwIP/xemacps 依赖 cache 操作 invalidate descriptor) */

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
    tcp_accept(pcb, rpc_accept);
    xil_printf("[srv] VD100 RPC server listen on 5000\r\n");
    return 0;
}

void transfer_data(void) {
    /* server 走 lwIP TCP recv callback 驱动, 不需要 main loop transfer */
}

void print_app_header(void) {
    xil_printf("\r\n%s\r\n", "VD100 RPC Server (Versal AI Edge VE2302)");
    xil_printf("TCP listen: 5000\r\n");
    xil_printf("HW: 3 ConvCore @ 100 MHz, INT8 LUT-MAC, generic RPC (no model burned in)\r\n");
    xil_printf("Cmds: LOAD_DDR / READ_DDR / POKE_CSR / PEEK_CSR / RUN_LAYERS / PING\r\n");
}
