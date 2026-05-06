`timescale 1ns/1ps

// =============================================================================
// axi_crossbar_4to4_sim.sv  --  Simplified 4M↔4S AXI4 crossbar (sim model)
//
// [Phase 7 SMC + NUMA] 用作 multicore_top_smc.sv 内部 placeholder, 等 Vivado
// SmartConnect IP (gen_axi_smc_4to4.tcl) 真生成出来后, 直接换成 IP 实例化即可.
//
// 行为模型, 跑 sim 用. 综合质量低, 不能上 FPGA.
//   - 每个 MI 一个 read-FSM + 一个 write-FSM, 锁定一个 SI owner 直到 transaction
//     (read: 至 rlast / write: 至 bvalid) 完成
//   - SI 之间不并发对同一 MI 抢: round-robin 选当前 IDLE 的 MI 上选最低 idx 的
//     pending SI
//   - 不同 MI 上的 transaction 可并发 (4 SI 各自路由到不同 MI 时全并发)
//   - SI 端单独跟踪 outstanding read 跟 outstanding write (allow read+write 并发)
//
// 地址路由 (跟 Syn/gen_axi_smc_4to4.tcl IP 配置一致):
//   addr[25:24] == 2'b00 → MI[0]  (全局 [0x0000_0000, 0x00FF_FFFF], 16 MB)
//   addr[25:24] == 2'b01 → MI[1]  (全局 [0x0100_0000, 0x01FF_FFFF])
//   addr[25:24] == 2'b10 → MI[2]  (全局 [0x0200_0000, 0x02FF_FFFF])
//   addr[25:24] == 2'b11 → MI[3]  (全局 [0x0300_0000, 0x03FF_FFFF])
// =============================================================================

module axi_crossbar_4to4_sim #(
    parameter int N        = 4,
    parameter int ADDR_W   = 32,
    parameter int DATA_W   = 128,
    parameter int ID_W     = 6,
    parameter int STRB_W   = DATA_W / 8
)(
    input  logic                       clk,
    input  logic                       rst_n,

    // ---- 4 SI (来自 4 ConvCore) ----
    input  logic [N*ID_W-1:0]          s_awid,
    input  logic [N*ADDR_W-1:0]        s_awaddr,
    input  logic [N*8-1:0]             s_awlen,
    input  logic [N*2-1:0]             s_awburst,
    input  logic [N-1:0]               s_awvalid,
    output logic [N-1:0]               s_awready,
    input  logic [N*DATA_W-1:0]        s_wdata,
    input  logic [N*STRB_W-1:0]        s_wstrb,
    input  logic [N-1:0]               s_wlast,
    input  logic [N-1:0]               s_wvalid,
    output logic [N-1:0]               s_wready,
    output logic [N*ID_W-1:0]          s_bid,
    output logic [N*2-1:0]             s_bresp,
    output logic [N-1:0]               s_bvalid,
    input  logic [N-1:0]               s_bready,
    input  logic [N*ID_W-1:0]          s_arid,
    input  logic [N*ADDR_W-1:0]        s_araddr,
    input  logic [N*8-1:0]             s_arlen,
    input  logic [N*2-1:0]             s_arburst,
    input  logic [N-1:0]               s_arvalid,
    output logic [N-1:0]               s_arready,
    output logic [N*ID_W-1:0]          s_rid,
    output logic [N*DATA_W-1:0]        s_rdata,
    output logic [N*2-1:0]             s_rresp,
    output logic [N-1:0]               s_rlast,
    output logic [N-1:0]               s_rvalid,
    input  logic [N-1:0]               s_rready,

    // ---- 4 MI (出 4 mem) ----
    output logic [N*ID_W-1:0]          m_awid,
    output logic [N*ADDR_W-1:0]        m_awaddr,
    output logic [N*8-1:0]             m_awlen,
    output logic [N*2-1:0]             m_awburst,
    output logic [N-1:0]               m_awvalid,
    input  logic [N-1:0]               m_awready,
    output logic [N*DATA_W-1:0]        m_wdata,
    output logic [N*STRB_W-1:0]        m_wstrb,
    output logic [N-1:0]               m_wlast,
    output logic [N-1:0]               m_wvalid,
    input  logic [N-1:0]               m_wready,
    input  logic [N*ID_W-1:0]          m_bid,
    input  logic [N*2-1:0]             m_bresp,
    input  logic [N-1:0]               m_bvalid,
    output logic [N-1:0]               m_bready,
    output logic [N*ID_W-1:0]          m_arid,
    output logic [N*ADDR_W-1:0]        m_araddr,
    output logic [N*8-1:0]             m_arlen,
    output logic [N*2-1:0]             m_arburst,
    output logic [N-1:0]               m_arvalid,
    input  logic [N-1:0]               m_arready,
    input  logic [N*ID_W-1:0]          m_rid,
    input  logic [N*DATA_W-1:0]        m_rdata,
    input  logic [N*2-1:0]             m_rresp,
    input  logic [N-1:0]               m_rlast,
    input  logic [N-1:0]               m_rvalid,
    output logic [N-1:0]               m_rready
);

    localparam int N_LOG = $clog2(N);

    // =========================================================================
    // 解 packed → unpacked, 方便循环写
    // =========================================================================
    logic [ID_W-1:0]    s_awid_u   [N];
    logic [ADDR_W-1:0]  s_awaddr_u [N];
    logic [7:0]         s_awlen_u  [N];
    logic [1:0]         s_awburst_u[N];
    logic [DATA_W-1:0]  s_wdata_u  [N];
    logic [STRB_W-1:0]  s_wstrb_u  [N];
    logic [ID_W-1:0]    s_arid_u   [N];
    logic [ADDR_W-1:0]  s_araddr_u [N];
    logic [7:0]         s_arlen_u  [N];
    logic [1:0]         s_arburst_u[N];

    logic [ID_W-1:0]    m_bid_u    [N];
    logic [1:0]         m_bresp_u  [N];
    logic [ID_W-1:0]    m_rid_u    [N];
    logic [DATA_W-1:0]  m_rdata_u  [N];
    logic [1:0]         m_rresp_u  [N];

    genvar gi;
    generate
        for (gi = 0; gi < N; gi++) begin : gen_unpack_si
            assign s_awid_u   [gi] = s_awid   [gi*ID_W   +: ID_W];
            assign s_awaddr_u [gi] = s_awaddr [gi*ADDR_W +: ADDR_W];
            assign s_awlen_u  [gi] = s_awlen  [gi*8      +: 8];
            assign s_awburst_u[gi] = s_awburst[gi*2      +: 2];
            assign s_wdata_u  [gi] = s_wdata  [gi*DATA_W +: DATA_W];
            assign s_wstrb_u  [gi] = s_wstrb  [gi*STRB_W +: STRB_W];
            assign s_arid_u   [gi] = s_arid   [gi*ID_W   +: ID_W];
            assign s_araddr_u [gi] = s_araddr [gi*ADDR_W +: ADDR_W];
            assign s_arlen_u  [gi] = s_arlen  [gi*8      +: 8];
            assign s_arburst_u[gi] = s_arburst[gi*2      +: 2];

            assign m_bid_u    [gi] = m_bid    [gi*ID_W   +: ID_W];
            assign m_bresp_u  [gi] = m_bresp  [gi*2      +: 2];
            assign m_rid_u    [gi] = m_rid    [gi*ID_W   +: ID_W];
            assign m_rdata_u  [gi] = m_rdata  [gi*DATA_W +: DATA_W];
            assign m_rresp_u  [gi] = m_rresp  [gi*2      +: 2];
        end
    endgenerate

    // =========================================================================
    // 路由解码: addr[25:24] = mem_id (4 个 16 MB region)
    // =========================================================================
    function automatic [1:0] route_id(input [ADDR_W-1:0] addr);
        return addr[25:24];
    endfunction

    // =========================================================================
    // Per-MI Read FSM: 锁定 SI owner 直到 rlast
    // =========================================================================
    typedef enum logic [1:0] {
        R_IDLE = 2'd0,
        R_AR   = 2'd1,    // 已选 owner, 把 ar 接给 MI, 等 arready
        R_R    = 2'd2     // ar 握手完, 等 r 数据流到 rlast
    } rstate_t;

    rstate_t            r_state    [N];
    logic [N_LOG-1:0]   r_owner    [N];   // 当前 SI owner

    // Per-MI Write FSM: 锁定 SI owner 直到 bvalid
    typedef enum logic [1:0] {
        W_IDLE = 2'd0,
        W_AW   = 2'd1,    // 把 aw 接给 MI, 等 awready
        W_W    = 2'd2,    // 把 w 接给 MI, 等 wlast
        W_B    = 2'd3     // 等 bvalid → 转给 owner
    } wstate_t;

    wstate_t            w_state    [N];
    logic [N_LOG-1:0]   w_owner    [N];

    // =========================================================================
    // SI 端 outstanding 标记: 防同 SI 多 outstanding 给 same MI
    //   busy_r_on_mi[s][m] = 1 表 SI[s] 正持有 MI[m] read transaction
    //   但 sim 简化: 一个 SI 同时对一个 MI 只 1 outstanding read + 1 write
    // 由于一个 SI 一次只能 arvalid 高一次 (周边 idma 等模块自身串行), 这里不需
    // 跟踪 SI 端复杂状态.
    // =========================================================================

    // ---- Read 端: 选 owner ----
    // 仅在 R_IDLE 选 owner: priority encoder 找第一个 SI 满足
    //   s_arvalid[s] && route(s_araddr_u[s]) == m && !其它 MI 已经选了 s 做 read
    logic [N-1:0] s_in_r_progress;       // bit s = 1 表 SI s 已被某 MI 选为 read owner
    always_comb begin
        s_in_r_progress = '0;
        for (int m = 0; m < N; m++) begin
            if (r_state[m] != R_IDLE) s_in_r_progress[r_owner[m]] = 1'b1;
        end
    end

    logic [N-1:0]      r_pick_vld     [N];
    logic [N_LOG-1:0]  r_pick_idx     [N];

    always_comb begin
        for (int m = 0; m < N; m++) begin
            r_pick_vld[m] = '0;
            r_pick_idx[m] = '0;
            for (int s = 0; s < N; s++) begin
                if (s_arvalid[s] && route_id(s_araddr_u[s]) == 2'(m)
                    && !s_in_r_progress[s] && !r_pick_vld[m]) begin
                    r_pick_vld[m] = 1'b1;
                    r_pick_idx[m] = N_LOG'(s);
                end
            end
        end
    end

    // ---- Write 端: 选 owner ----
    logic [N-1:0] s_in_w_progress;
    always_comb begin
        s_in_w_progress = '0;
        for (int m = 0; m < N; m++) begin
            if (w_state[m] != W_IDLE) s_in_w_progress[w_owner[m]] = 1'b1;
        end
    end

    logic [N-1:0]      w_pick_vld     [N];
    logic [N_LOG-1:0]  w_pick_idx     [N];

    always_comb begin
        for (int m = 0; m < N; m++) begin
            w_pick_vld[m] = '0;
            w_pick_idx[m] = '0;
            for (int s = 0; s < N; s++) begin
                if (s_awvalid[s] && route_id(s_awaddr_u[s]) == 2'(m)
                    && !s_in_w_progress[s] && !w_pick_vld[m]) begin
                    w_pick_vld[m] = 1'b1;
                    w_pick_idx[m] = N_LOG'(s);
                end
            end
        end
    end

    // =========================================================================
    // FSM 推进
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int m = 0; m < N; m++) begin
                r_state[m] <= R_IDLE;
                w_state[m] <= W_IDLE;
                r_owner[m] <= '0;
                w_owner[m] <= '0;
            end
        end else begin
            for (int m = 0; m < N; m++) begin
                // ---- Read FSM ----
                case (r_state[m])
                    R_IDLE: if (r_pick_vld[m]) begin
                        r_owner[m] <= r_pick_idx[m];
                        r_state[m] <= R_AR;
                    end
                    R_AR:   if (m_arvalid[m] && m_arready[m]) r_state[m] <= R_R;
                    R_R:    if (m_rvalid[m] && m_rready[m] && m_rlast[m])
                                                              r_state[m] <= R_IDLE;
                    default:                                  r_state[m] <= R_IDLE;
                endcase

                // ---- Write FSM ----
                case (w_state[m])
                    W_IDLE: if (w_pick_vld[m]) begin
                        w_owner[m] <= w_pick_idx[m];
                        w_state[m] <= W_AW;
                    end
                    W_AW:   if (m_awvalid[m] && m_awready[m]) w_state[m] <= W_W;
                    W_W:    if (m_wvalid[m] && m_wready[m] && m_wlast[m])
                                                              w_state[m] <= W_B;
                    W_B:    if (m_bvalid[m] && m_bready[m])   w_state[m] <= W_IDLE;
                    default:                                  w_state[m] <= W_IDLE;
                endcase
            end
        end
    end

    // =========================================================================
    // 路由 SI → MI (Read AR/R)
    //   r_state[m] == R_AR: 把 SI[r_owner[m]].ar 接给 MI[m]
    //   r_state[m] == R_R : 把 MI[m].r 接给 SI[r_owner[m]]
    // =========================================================================
    logic [ID_W-1:0]   m_arid_u   [N];
    logic [ADDR_W-1:0] m_araddr_u [N];
    logic [7:0]        m_arlen_u  [N];
    logic [1:0]        m_arburst_u[N];
    logic              m_arvalid_u[N];

    always_comb begin
        for (int m = 0; m < N; m++) begin
            if (r_state[m] == R_AR) begin
                m_arid_u   [m] = s_arid_u   [r_owner[m]];
                m_araddr_u [m] = s_araddr_u [r_owner[m]];
                m_arlen_u  [m] = s_arlen_u  [r_owner[m]];
                m_arburst_u[m] = s_arburst_u[r_owner[m]];
                m_arvalid_u[m] = 1'b1;
            end else begin
                m_arid_u   [m] = '0;
                m_araddr_u [m] = '0;
                m_arlen_u  [m] = '0;
                m_arburst_u[m] = '0;
                m_arvalid_u[m] = 1'b0;
            end
        end
    end

    // SI[s].arready: 仅当某 MI 处于 R_AR 状态且 owner 是 s 且 MI 端 arready 高
    logic [N-1:0] s_arready_u;
    always_comb begin
        for (int s = 0; s < N; s++) begin
            s_arready_u[s] = 1'b0;
            for (int m = 0; m < N; m++) begin
                if (r_state[m] == R_AR && r_owner[m] == N_LOG'(s) && m_arready[m])
                    s_arready_u[s] = 1'b1;
            end
        end
    end

    // SI[s].r*: 找服务自己的 MI
    logic [ID_W-1:0]   s_rid_u   [N];
    logic [DATA_W-1:0] s_rdata_u [N];
    logic [1:0]        s_rresp_u [N];
    logic              s_rlast_u [N];
    logic              s_rvalid_u[N];
    logic              m_rready_u[N];

    always_comb begin
        for (int s = 0; s < N; s++) begin
            s_rid_u   [s] = '0;
            s_rdata_u [s] = '0;
            s_rresp_u [s] = '0;
            s_rlast_u [s] = 1'b0;
            s_rvalid_u[s] = 1'b0;
            for (int m = 0; m < N; m++) begin
                if (r_state[m] == R_R && r_owner[m] == N_LOG'(s)) begin
                    s_rid_u   [s] = m_rid_u   [m];
                    s_rdata_u [s] = m_rdata_u [m];
                    s_rresp_u [s] = m_rresp_u [m];
                    s_rlast_u [s] = m_rlast   [m];
                    s_rvalid_u[s] = m_rvalid  [m];
                end
            end
        end

        for (int m = 0; m < N; m++) begin
            if (r_state[m] == R_R)
                m_rready_u[m] = s_rready[r_owner[m]];
            else
                m_rready_u[m] = 1'b0;
        end
    end

    // =========================================================================
    // 路由 SI → MI (Write AW/W/B)
    // =========================================================================
    logic [ID_W-1:0]   m_awid_u   [N];
    logic [ADDR_W-1:0] m_awaddr_u [N];
    logic [7:0]        m_awlen_u  [N];
    logic [1:0]        m_awburst_u[N];
    logic              m_awvalid_u[N];
    logic [DATA_W-1:0] m_wdata_u  [N];
    logic [STRB_W-1:0] m_wstrb_u  [N];
    logic              m_wlast_u  [N];
    logic              m_wvalid_u [N];
    logic              m_bready_u [N];

    always_comb begin
        for (int m = 0; m < N; m++) begin
            if (w_state[m] == W_AW) begin
                m_awid_u   [m] = s_awid_u   [w_owner[m]];
                m_awaddr_u [m] = s_awaddr_u [w_owner[m]];
                m_awlen_u  [m] = s_awlen_u  [w_owner[m]];
                m_awburst_u[m] = s_awburst_u[w_owner[m]];
                m_awvalid_u[m] = 1'b1;
            end else begin
                m_awid_u   [m] = '0;
                m_awaddr_u [m] = '0;
                m_awlen_u  [m] = '0;
                m_awburst_u[m] = '0;
                m_awvalid_u[m] = 1'b0;
            end

            if (w_state[m] == W_W) begin
                m_wdata_u  [m] = s_wdata_u  [w_owner[m]];
                m_wstrb_u  [m] = s_wstrb_u  [w_owner[m]];
                m_wlast_u  [m] = s_wlast    [w_owner[m]];
                m_wvalid_u [m] = s_wvalid   [w_owner[m]];
            end else begin
                m_wdata_u  [m] = '0;
                m_wstrb_u  [m] = '0;
                m_wlast_u  [m] = 1'b0;
                m_wvalid_u [m] = 1'b0;
            end

            m_bready_u[m] = (w_state[m] == W_B) ? s_bready[w_owner[m]] : 1'b0;
        end
    end

    logic [N-1:0]      s_awready_u;
    logic [N-1:0]      s_wready_u;
    logic [ID_W-1:0]   s_bid_u    [N];
    logic [1:0]        s_bresp_u  [N];
    logic [N-1:0]      s_bvalid_u;

    always_comb begin
        s_awready_u = '0;
        s_wready_u  = '0;
        s_bvalid_u  = '0;
        for (int s = 0; s < N; s++) begin
            s_bid_u   [s] = '0;
            s_bresp_u [s] = '0;
            for (int m = 0; m < N; m++) begin
                if (w_state[m] == W_AW && w_owner[m] == N_LOG'(s) && m_awready[m])
                    s_awready_u[s] = 1'b1;
                if (w_state[m] == W_W  && w_owner[m] == N_LOG'(s) && m_wready[m])
                    s_wready_u[s]  = 1'b1;
                if (w_state[m] == W_B  && w_owner[m] == N_LOG'(s)) begin
                    s_bid_u   [s] = m_bid_u   [m];
                    s_bresp_u [s] = m_bresp_u [m];
                    s_bvalid_u[s] = m_bvalid  [m];
                end
            end
        end
    end

    // =========================================================================
    // pack 回外部 packed 接口
    // =========================================================================
    generate
        for (gi = 0; gi < N; gi++) begin : gen_pack_mi
            assign m_arid   [gi*ID_W   +: ID_W]   = m_arid_u   [gi];
            assign m_araddr [gi*ADDR_W +: ADDR_W] = m_araddr_u [gi];
            assign m_arlen  [gi*8      +: 8]      = m_arlen_u  [gi];
            assign m_arburst[gi*2      +: 2]      = m_arburst_u[gi];
            assign m_arvalid[gi]                  = m_arvalid_u[gi];
            assign m_rready [gi]                  = m_rready_u [gi];

            assign m_awid   [gi*ID_W   +: ID_W]   = m_awid_u   [gi];
            assign m_awaddr [gi*ADDR_W +: ADDR_W] = m_awaddr_u [gi];
            assign m_awlen  [gi*8      +: 8]      = m_awlen_u  [gi];
            assign m_awburst[gi*2      +: 2]      = m_awburst_u[gi];
            assign m_awvalid[gi]                  = m_awvalid_u[gi];
            assign m_wdata  [gi*DATA_W +: DATA_W] = m_wdata_u  [gi];
            assign m_wstrb  [gi*STRB_W +: STRB_W] = m_wstrb_u  [gi];
            assign m_wlast  [gi]                  = m_wlast_u  [gi];
            assign m_wvalid [gi]                  = m_wvalid_u [gi];
            assign m_bready [gi]                  = m_bready_u [gi];

            assign s_arready[gi]                  = s_arready_u[gi];
            assign s_rid    [gi*ID_W   +: ID_W]   = s_rid_u    [gi];
            assign s_rdata  [gi*DATA_W +: DATA_W] = s_rdata_u  [gi];
            assign s_rresp  [gi*2      +: 2]      = s_rresp_u  [gi];
            assign s_rlast  [gi]                  = s_rlast_u  [gi];
            assign s_rvalid [gi]                  = s_rvalid_u [gi];

            assign s_awready[gi]                  = s_awready_u[gi];
            assign s_wready [gi]                  = s_wready_u [gi];
            assign s_bid    [gi*ID_W   +: ID_W]   = s_bid_u    [gi];
            assign s_bresp  [gi*2      +: 2]      = s_bresp_u  [gi];
            assign s_bvalid [gi]                  = s_bvalid_u [gi];
        end
    endgenerate

endmodule
