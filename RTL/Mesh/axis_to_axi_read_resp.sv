`timescale 1ns/1ps

// =============================================================================
// axis_to_axi_read_resp.sv  --  mesh READ_RESP packet → ConvCore AXI master read resp
//
// NUMA 模式下 ConvCore 起 read burst 后, mem 端回 READ_RESP packet 经 mesh 路由
// 回来. axi_reader_to_axis 已经按 mem 边界把单个 AR burst 拆成多个 READ_REQ packet,
// 每个 mem 各回一个 RESP packet. 本模块**合并多个 packet 数据流成单 AXI burst response**:
//   - 按 cfg_total_beats (= arlen+1) 计数 m_axi_r 拉的 beats
//   - 累计达到 total_beats 时拉 m_axi_rlast (无视 packet 边界)
//
// Packet 协议:
//   flit 0:    header (opcode=0x2, addr20, burst_len, return_id)
//   flit 1..N: data
//
// 状态 (per packet):
//   S_HEADER: 收 1 拍 header → latch packet burst_len → 进 S_DATA
//   S_DATA  : forward N data 到 m_axi_r*, 收完该 packet 的 N data 后归 S_HEADER 等下一 packet
//             (跨 packet 时 r_total_recv 不重置, 累计到 cfg_total_beats 才拉 rlast)
// =============================================================================

module axis_to_axi_read_resp #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ID_W   = 8
)(
    input  logic                 clk,
    input  logic                 rst_n,

    // ---- mesh AXIS slave (收 READ_RESP packet) ----
    input  logic                 s_axis_tvalid,
    output logic                 s_axis_tready,
    input  logic [DATA_W-1:0]    s_axis_tdata,
    input  logic                 s_axis_tlast,
    input  logic [DEST_W-1:0]    s_axis_tdest,

    // ---- AXI master read response 端口 (ConvCore.bus_r 出) ----
    output logic [ID_W-1:0]      m_axi_rid,
    output logic [DATA_W-1:0]    m_axi_rdata,
    output logic [1:0]           m_axi_rresp,
    output logic                 m_axi_rlast,
    output logic                 m_axi_rvalid,
    input  logic                 m_axi_rready,

    // ---- 跟 axi_reader_to_axis 同步: 跟踪当前 outstanding burst 的 ARID 跟总 beats ----
    input  logic [ID_W-1:0]      cfg_outstanding_arid,
    input  logic [15:0]          cfg_total_beats           // = arlen+1, 用于跨 packet 累计计 rlast
);

    typedef enum logic [0:0] { S_HEADER, S_DATA } state_t;
    state_t st;

    logic [15:0] r_total_recv;     // 累计跨 packet 的 m_axi_r beats (重置时机: rlast fire)

    // s_axis_tready:
    //   S_HEADER: ready (吃 header)
    //   S_DATA:   m_axi_rready 时 ready (back-pressure)
    assign s_axis_tready = (st == S_HEADER) || ((st == S_DATA) && m_axi_rready);

    // m_axi_r 输出
    assign m_axi_rvalid = (st == S_DATA) && s_axis_tvalid;
    assign m_axi_rdata  = s_axis_tdata;
    assign m_axi_rresp  = 2'b00;
    assign m_axi_rid    = cfg_outstanding_arid;
    // rlast 在 r_total_recv 累计 = cfg_total_beats 时拉 (= 累计跨 packet 的最后 1 拍)
    assign m_axi_rlast  = (st == S_DATA) && s_axis_tvalid &&
                          (r_total_recv + 16'd1 == cfg_total_beats);

    state_t st_next;
    always_comb begin
        st_next = st;
        case (st)
            S_HEADER: if (s_axis_tvalid && s_axis_tready) st_next = S_DATA;
            // packet 内 axis_tlast 表示该 packet 数据流完了, 切回 S_HEADER 等下一 packet
            S_DATA  : if (m_axi_rvalid && m_axi_rready && s_axis_tlast) st_next = S_HEADER;
            default : st_next = S_HEADER;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) st <= S_HEADER;
        else        st <= st_next;
    end

    // r_total_recv: m_axi_rlast 时清, 否则每 fire 一拍累加 (跨 packet 持续累)
    always_ff @(posedge clk) begin
        if      (!rst_n)                                          r_total_recv <= '0;
        else if (m_axi_rvalid && m_axi_rready && m_axi_rlast)     r_total_recv <= '0;
        else if (m_axi_rvalid && m_axi_rready)                    r_total_recv <= r_total_recv + 16'd1;
    end

endmodule
