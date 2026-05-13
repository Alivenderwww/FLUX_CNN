`timescale 1ns/1ps

// =============================================================================
// mm2s_arb.sv  --  AXI DataMover MM2S 通道仲裁器 (4 路: idma + wdma + rdma + ocmd)
//
// Pipelined: 利用 DataMover 内部 cmd FIFO 让多 cmd in-flight. cmd 严格按入序
// 处理, 因此 data / sts 的目的端 (owner) 只需用 FIFO 跟踪 cmd 入序即可.
//
// 仲裁优先级 (高→低): idma > rdma > ocmd > wdma
//   - idma: 流式输入, 实时性最敏感
//   - rdma: 残差/bias 加载, layer 启动期一次性 (只有 1 条 cmd)
//   - ocmd: ODMA SG dispatcher 拉 cmd list (Phase 7 SMC + NUMA, 频繁但短)
//   - wdma: 权重加载, layer 启动期一次性
// rdma + wdma 都是 batch, 但 rdma 需要在 layer compute 开始前完成 (bias_rf
// 依赖), 略优先于 wdma. ocmd 介于两者之间: 频度高 (每行 OFM 1-N 条 cmd),
// 单次很短 (1 beat = 16 byte cmd 元数据).
//
// Round E (2026-05-07): 加 WDMA 饥饿提优先级机制. WDMA 等待 ≥ STARVE_THRESH
// 拍后强制 grant 一次 (跳到优先级最高), 防 IDMA 持续占用 mm2s 让 WDMA 饿死.
// 实测 Round B baseline ResNet11 L0 wgt_st=10500 / 总 cy=45886, 23% cycle WDMA 等
// IDMA 让位. 加饥饿机制让 wdma 周期性插入 ~每 32 cy 抢一次, 减少 mac_array
// stall (act_id 跟 wgt_st 配对掉同样多).
//
// data / sts 各一个 owner FIFO (深度 8). owner 编码: 2'd0=idma, 2'd1=wdma,
// 2'd2=rdma, 2'd3=ocmd. push on cmd_fire, pop 各异步.
//
// SG_MODE=0 兼容: 原 IDMA path 不变, ocmd 路 tie 0 即可 (4-th port 没 valid 不
// 影响其它路仲裁).
// =============================================================================

module mm2s_arb #(
    parameter int DATA_W      = 128,
    parameter int OFIFO_DEPTH = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- 端口 0: idma_ctrl ----
    input  logic                    idma_cmd_tvalid,
    output logic                    idma_cmd_tready,
    input  logic [71:0]             idma_cmd_tdata,
    output logic                    idma_data_tvalid,
    input  logic                    idma_data_tready,
    output logic [DATA_W-1:0]       idma_data_tdata,
    output logic [DATA_W/8-1:0]     idma_data_tkeep,
    output logic                    idma_data_tlast,
    output logic                    idma_sts_tvalid,
    input  logic                    idma_sts_tready,
    output logic [7:0]              idma_sts_tdata,

    // ---- 端口 1: wdma_ctrl ----
    input  logic                    wdma_cmd_tvalid,
    output logic                    wdma_cmd_tready,
    input  logic [71:0]             wdma_cmd_tdata,
    output logic                    wdma_data_tvalid,
    input  logic                    wdma_data_tready,
    output logic [DATA_W-1:0]       wdma_data_tdata,
    output logic [DATA_W/8-1:0]     wdma_data_tkeep,
    output logic                    wdma_data_tlast,
    output logic                    wdma_sts_tvalid,
    input  logic                    wdma_sts_tready,
    output logic [7:0]              wdma_sts_tdata,

    // ---- 端口 2: rdma_ctrl ----
    input  logic                    rdma_cmd_tvalid,
    output logic                    rdma_cmd_tready,
    input  logic [71:0]             rdma_cmd_tdata,
    output logic                    rdma_data_tvalid,
    input  logic                    rdma_data_tready,
    output logic [DATA_W-1:0]       rdma_data_tdata,
    output logic [DATA_W/8-1:0]     rdma_data_tkeep,
    output logic                    rdma_data_tlast,
    output logic                    rdma_sts_tvalid,
    input  logic                    rdma_sts_tready,
    output logic [7:0]              rdma_sts_tdata,

    // ---- 端口 3: ocmd (ODMA SG dispatcher 拉 cmd list, SG_MODE 用; SG_MODE=0 tie 0) ----
    input  logic                    ocmd_cmd_tvalid,
    output logic                    ocmd_cmd_tready,
    input  logic [71:0]             ocmd_cmd_tdata,
    output logic                    ocmd_data_tvalid,
    input  logic                    ocmd_data_tready,
    output logic [DATA_W-1:0]       ocmd_data_tdata,
    output logic [DATA_W/8-1:0]     ocmd_data_tkeep,
    output logic                    ocmd_data_tlast,
    output logic                    ocmd_sts_tvalid,
    input  logic                    ocmd_sts_tready,
    output logic [7:0]              ocmd_sts_tdata,

    // ---- 上游: axi_dm.MM2S ----
    output logic                    mm2s_cmd_tvalid,
    input  logic                    mm2s_cmd_tready,
    output logic [71:0]             mm2s_cmd_tdata,
    input  logic                    mm2s_data_tvalid,
    output logic                    mm2s_data_tready,
    input  logic [DATA_W-1:0]       mm2s_data_tdata,
    input  logic [DATA_W/8-1:0]     mm2s_data_tkeep,
    input  logic                    mm2s_data_tlast,
    input  logic                    mm2s_sts_tvalid,
    output logic                    mm2s_sts_tready,
    input  logic [7:0]              mm2s_sts_tdata
);

    localparam int OWN_W = 2;     // 0=idma, 1=wdma, 2=rdma, 3=ocmd
    localparam int PTR_W = $clog2(OFIFO_DEPTH);
    localparam int CNT_W = $clog2(OFIFO_DEPTH + 1);

    // Round E: WDMA 饥饿门槛 (cycle 数). 等 ≥ STARVE_THRESH 拍没 grant → 提优先级到最高.
    // 选 32 是个折中: 太短让 WDMA 频繁打断 IDMA, 太长 WDMA 仍饿. 实测可调.
    localparam int STARVE_THRESH = 32;

    // =========================================================================
    // WDMA 饥饿计数 (Round E)
    //   wdma 有 cmd 等且没 grant → cnt += 1
    //   wdma fire → cnt 清零
    //   wdma 没 cmd → cnt 清零 (没活就不算饿)
    // =========================================================================
    logic [$clog2(STARVE_THRESH+1)-1:0] r_wdma_wait_cnt;
    logic wdma_starve;
    assign wdma_starve = (r_wdma_wait_cnt >= STARVE_THRESH);

    // =========================================================================
    // CMD 仲裁: 默认 idma > rdma > ocmd > wdma. WDMA 饿到阈值时强制 grant 一次.
    // =========================================================================
    logic [OWN_W-1:0] cmd_owner;
    always_comb begin
        if      (wdma_starve && wdma_cmd_tvalid) cmd_owner = 2'd1;   // ★ WDMA 饿了优先抢
        else if (idma_cmd_tvalid) cmd_owner = 2'd0;
        else if (rdma_cmd_tvalid) cmd_owner = 2'd2;
        else if (ocmd_cmd_tvalid) cmd_owner = 2'd3;
        else                       cmd_owner = 2'd1;     // wdma 默认
    end

    logic any_cmd_tvalid;
    assign any_cmd_tvalid = idma_cmd_tvalid | wdma_cmd_tvalid | rdma_cmd_tvalid | ocmd_cmd_tvalid;

    logic data_full, sts_full;
    logic data_empty_w, sts_empty_w;   // forward decl for serialization gate

    always_comb begin
        case (cmd_owner)
            2'd0   : mm2s_cmd_tdata = idma_cmd_tdata;
            2'd1   : mm2s_cmd_tdata = wdma_cmd_tdata;
            2'd2   : mm2s_cmd_tdata = rdma_cmd_tdata;
            2'd3   : mm2s_cmd_tdata = ocmd_cmd_tdata;
            default: mm2s_cmd_tdata = idma_cmd_tdata;
        endcase
    end
    // VD100 fix 2026-05-13: 仅 data single-outstanding (data_empty_w gate), 去掉 sts gate.
    // axi_dm 在板上偶尔不出 sts 让 sts_cnt 累积 → sts_empty_w=0 永久 → cmd_tvalid 永卡.
    // 配合 v29 dispatcher r_done 不依赖 sts, 整链 sts 信号已彻底无用.
    // sts_full 也去掉 — 反正 dispatcher 不消费 sts, FIFO 写 wrap 不引发问题.
    assign mm2s_cmd_tvalid = any_cmd_tvalid && !data_full && data_empty_w;

    logic cmd_fire;
    assign cmd_fire = mm2s_cmd_tvalid && mm2s_cmd_tready;

    assign idma_cmd_tready = (cmd_owner == 2'd0) && idma_cmd_tvalid && mm2s_cmd_tready
                          && !data_full && data_empty_w;
    assign wdma_cmd_tready = (cmd_owner == 2'd1) && wdma_cmd_tvalid && mm2s_cmd_tready
                          && !data_full && data_empty_w;
    assign rdma_cmd_tready = (cmd_owner == 2'd2) && rdma_cmd_tvalid && mm2s_cmd_tready
                          && !data_full && data_empty_w;
    assign ocmd_cmd_tready = (cmd_owner == 2'd3) && ocmd_cmd_tvalid && mm2s_cmd_tready
                          && !data_full && data_empty_w;

    // =========================================================================
    // r_wdma_wait_cnt: WDMA 累计等待 cycle (Round E)
    //   - WDMA 有 cmd 等待但 cmd_owner 不是 WDMA → cnt += 1
    //   - WDMA fire (cmd_tready 拨高那拍) → cnt 清零
    //   - WDMA 没 cmd → cnt 清零 (idle 不算饿)
    //   计数饱和到 STARVE_THRESH (再涨没意义).
    // =========================================================================
    logic wdma_fire;
    assign wdma_fire = wdma_cmd_tvalid && wdma_cmd_tready;
    always_ff @(posedge clk) begin
        if (!rst_n) r_wdma_wait_cnt <= '0;
        else if (wdma_fire || !wdma_cmd_tvalid) r_wdma_wait_cnt <= '0;
        else if (r_wdma_wait_cnt < STARVE_THRESH) r_wdma_wait_cnt <= r_wdma_wait_cnt + 1'b1;
    end

    // =========================================================================
    // 性能 counter (论文素材, 不影响功能)
    //   hs_idma_grant: idma 拿到 mm2s 通道总 cy
    //   hs_wdma_grant: wdma 拿到 (含 starve 抢占)
    //   hs_rdma_grant: rdma 拿到
    //   hs_ocmd_grant: ocmd 拿到
    //   hs_starve_preempt: WDMA starve 触发抢占次数
    //   hs_data_full_stall: data FIFO 满阻塞 cmd_fire 次数
    // =========================================================================
    logic [31:0] hs_idma_grant, hs_wdma_grant, hs_rdma_grant, hs_ocmd_grant;
    logic [31:0] hs_starve_preempt, hs_data_full_stall;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_idma_grant      <= '0;
            hs_wdma_grant      <= '0;
            hs_rdma_grant      <= '0;
            hs_ocmd_grant      <= '0;
            hs_starve_preempt  <= '0;
            hs_data_full_stall <= '0;
        end else begin
            // 仅在 cmd_fire 那拍统计 (实际 grant 那拍)
            if (cmd_fire) begin
                case (cmd_owner)
                    2'd0: hs_idma_grant <= hs_idma_grant + 32'd1;
                    2'd1: hs_wdma_grant <= hs_wdma_grant + 32'd1;
                    2'd2: hs_rdma_grant <= hs_rdma_grant + 32'd1;
                    2'd3: hs_ocmd_grant <= hs_ocmd_grant + 32'd1;
                endcase
            end
            // wdma starve 触发抢占 (cmd_owner=1 因为 starve 而非默认)
            if (cmd_fire && (cmd_owner == 2'd1) && wdma_starve && idma_cmd_tvalid) begin
                hs_starve_preempt <= hs_starve_preempt + 32'd1;
            end
            // data FIFO 满阻塞
            if (any_cmd_tvalid && data_full) hs_data_full_stall <= hs_data_full_stall + 32'd1;
        end
    end

    // =========================================================================
    // owner FIFO × 2 (data / sts), each depth = OFIFO_DEPTH, width = OWN_W
    // =========================================================================
    logic [OWN_W-1:0]  data_mem [0:OFIFO_DEPTH-1];
    logic [OWN_W-1:0]  sts_mem  [0:OFIFO_DEPTH-1];
    logic [PTR_W-1:0]  data_wr, data_rd;
    logic [PTR_W-1:0]  sts_wr,  sts_rd;
    logic [CNT_W-1:0]  data_cnt, sts_cnt;
    logic              data_empty, sts_empty;
    logic [OWN_W-1:0]  data_head, sts_head;

    assign data_empty = (data_cnt == '0);
    assign data_empty_w = data_empty;   // wire to upstream gate
    assign data_full  = (data_cnt == CNT_W'(OFIFO_DEPTH));
    assign data_head  = data_mem[data_rd];

    assign sts_empty  = (sts_cnt == '0);
    assign sts_empty_w = sts_empty;     // wire to upstream gate
    assign sts_full   = (sts_cnt == CNT_W'(OFIFO_DEPTH));
    assign sts_head   = sts_mem[sts_rd];

    logic data_tlast_fire, sts_fire;
    assign data_tlast_fire = mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast;
    assign sts_fire        = mm2s_sts_tvalid  && mm2s_sts_tready;

    // VD100 dbg 2026-05-12: 追 cmd/data/sts fire 时序看 axi_dm 行为
    `ifdef MM2S_ARB_DBG
    int dbg_cmd_idx, dbg_data_tlast_idx, dbg_sts_idx;
    always_ff @(posedge clk) begin
        if (!rst_n) begin dbg_cmd_idx <= 0; dbg_data_tlast_idx <= 0; dbg_sts_idx <= 0; end
        else begin
            if (cmd_fire) begin
                $display("[%0t] [%m] cmd_fire #%0d owner=%0d data_cnt=%0d sts_cnt=%0d", $time, dbg_cmd_idx, cmd_owner, data_cnt, sts_cnt);
                dbg_cmd_idx <= dbg_cmd_idx + 1;
            end
            if (data_tlast_fire) begin
                $display("[%0t] [%m] data_tlast_fire #%0d data_head=%0d data_cnt=%0d", $time, dbg_data_tlast_idx, data_head, data_cnt);
                dbg_data_tlast_idx <= dbg_data_tlast_idx + 1;
            end
            if (sts_fire) begin
                $display("[%0t] [%m] sts_fire #%0d sts_head=%0d sts_cnt=%0d", $time, dbg_sts_idx, sts_head, sts_cnt);
                dbg_sts_idx <= dbg_sts_idx + 1;
            end
        end
    end
    `endif


    always_ff @(posedge clk) begin
        if (!rst_n) begin
            data_wr  <= '0;
            data_rd  <= '0;
            data_cnt <= '0;
        end else begin
            if (cmd_fire) begin
                data_mem[data_wr] <= cmd_owner;
                data_wr           <= data_wr + 1'b1;
            end
            if (data_tlast_fire) data_rd <= data_rd + 1'b1;
            case ({cmd_fire, data_tlast_fire})
                2'b10  : data_cnt <= data_cnt + 1'b1;
                2'b01  : data_cnt <= data_cnt - 1'b1;
                default: data_cnt <= data_cnt;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sts_wr  <= '0;
            sts_rd  <= '0;
            sts_cnt <= '0;
        end else begin
            if (cmd_fire) begin
                sts_mem[sts_wr] <= cmd_owner;
                sts_wr          <= sts_wr + 1'b1;
            end
            if (sts_fire) sts_rd <= sts_rd + 1'b1;
            case ({cmd_fire, sts_fire})
                2'b10  : sts_cnt <= sts_cnt + 1'b1;
                2'b01  : sts_cnt <= sts_cnt - 1'b1;
                default: sts_cnt <= sts_cnt;
            endcase
        end
    end

    // =========================================================================
    // DATA demux (按 data_head)
    // =========================================================================
    assign idma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd0);
    assign idma_data_tdata  = mm2s_data_tdata;
    assign idma_data_tkeep  = mm2s_data_tkeep;
    assign idma_data_tlast  = mm2s_data_tlast;

    assign wdma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd1);
    assign wdma_data_tdata  = mm2s_data_tdata;
    assign wdma_data_tkeep  = mm2s_data_tkeep;
    assign wdma_data_tlast  = mm2s_data_tlast;

    assign rdma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd2);
    assign rdma_data_tdata  = mm2s_data_tdata;
    assign rdma_data_tkeep  = mm2s_data_tkeep;
    assign rdma_data_tlast  = mm2s_data_tlast;

    assign ocmd_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd3);
    assign ocmd_data_tdata  = mm2s_data_tdata;
    assign ocmd_data_tkeep  = mm2s_data_tkeep;
    assign ocmd_data_tlast  = mm2s_data_tlast;

    always_comb begin
        case (data_head)
            2'd0   : mm2s_data_tready = !data_empty && idma_data_tready;
            2'd1   : mm2s_data_tready = !data_empty && wdma_data_tready;
            2'd2   : mm2s_data_tready = !data_empty && rdma_data_tready;
            2'd3   : mm2s_data_tready = !data_empty && ocmd_data_tready;
            default: mm2s_data_tready = 1'b0;
        endcase
    end

    // =========================================================================
    // STS demux (按 sts_head)
    // =========================================================================
    assign idma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd0);
    assign idma_sts_tdata  = mm2s_sts_tdata;

    assign wdma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd1);
    assign wdma_sts_tdata  = mm2s_sts_tdata;

    assign rdma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd2);
    assign rdma_sts_tdata  = mm2s_sts_tdata;

    assign ocmd_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd3);
    assign ocmd_sts_tdata  = mm2s_sts_tdata;

    always_comb begin
        case (sts_head)
            2'd0   : mm2s_sts_tready = !sts_empty && idma_sts_tready;
            2'd1   : mm2s_sts_tready = !sts_empty && wdma_sts_tready;
            2'd2   : mm2s_sts_tready = !sts_empty && rdma_sts_tready;
            2'd3   : mm2s_sts_tready = !sts_empty && ocmd_sts_tready;
            default: mm2s_sts_tready = 1'b0;
        endcase
    end

endmodule
