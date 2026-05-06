`timescale 1ns/1ps

// =============================================================================
// axis_packet_read.sv  --  mem 端被读引擎 (NUMA 模式)
//
// NUMA 模式下 ConvCore 主动用全局地址发起 read burst, 经 axi_reader_to_axis 桥
// 包成 READ_REQ packet 路由到对应 mem. 本模块在 mem 端处理 REQ:
//   1. 收 READ_REQ packet header (opcode=0x1)
//   2. 解析 addr20 / burst_len / return_id
//   3. 顺序从 ddr_mem 读 burst_len 个 word
//   4. 组成 READ_RESP packet (header + N data flit), 发回 mesh
//
// REQ packet:
//   flit 0 only (header), opcode=0x1, addr20, burst_len, return_id, tlast=1
//
// RESP packet:
//   flit 0:    header (opcode=0x2, addr20 复制, burst_len 复制, tdest = return_id)
//   flit 1..N: data (= ddr_mem[addr20+i])
//   flit N:    tlast=1 (= AXI rlast)
//
// 跟 axis_packet_rx 共享 mesh slave 端口? 本模块只看 REQ packet (opcode=0x1),
// 实际 wrapper 在收 packet 时按 opcode 分流给 packet_rx (WRITE) 或 packet_read (READ).
// 这里假设输入只来 READ_REQ packet (上层已分流).
// =============================================================================

module axis_packet_read #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ADDR_W = 32       // ddr_mem 地址宽度
)(
    input  logic                clk,
    input  logic                rst_n,

    // AXIS slave: 收 READ_REQ packet (上层 demux 后只送 opcode=0x1)
    input  logic                s_axis_tvalid,
    output logic                s_axis_tready,
    input  logic [DATA_W-1:0]   s_axis_tdata,
    input  logic                s_axis_tlast,
    input  logic [DEST_W-1:0]   s_axis_tdest,

    // ddr_mem 读端口
    output logic                ddr_re,
    output logic [ADDR_W-1:0]   ddr_raddr,
    input  logic [DATA_W-1:0]   ddr_rdata,    // 1 拍延迟回数据 (上层用 reg 同步)

    // AXIS master: 发 READ_RESP packet
    output logic                m_axis_tvalid,
    input  logic                m_axis_tready,
    output logic [DATA_W-1:0]   m_axis_tdata,
    output logic                m_axis_tlast,
    output logic [DEST_W-1:0]   m_axis_tdest
);

    // FSM:
    //   S_IDLE     : 等 REQ packet
    //   S_LATCH    : 收到 REQ header, latch 字段, 准备发 RESP
    //   S_HEADER   : 发 RESP header (1 flit)
    //   S_FETCH    : 拉 ddr_re 读第 i 个 data, 1 拍后 ddr_rdata 有效
    //   S_DATA     : 把 ddr_rdata forward 到 m_axis (含 tlast = i==N-1)
    typedef enum logic [2:0] {
        S_IDLE   = 3'd0,
        S_HEADER = 3'd1,
        S_FETCH  = 3'd2,
        S_DATA   = 3'd3
    } state_t;
    state_t st;

    logic [3:0]         r_req_opcode;       // 不用, 但 latch 方便 debug
    logic [19:0]        r_req_addr20;
    logic [15:0]        r_req_burst_len;
    logic [DEST_W-1:0]  r_req_return_id;
    logic [15:0]        r_data_idx;         // 当前发到第几个 data flit (0..burst_len-1)

    // ---- AXIS slave (收 REQ) ----
    assign s_axis_tready = (st == S_IDLE);

    // S_IDLE 收 REQ header (1 flit, tlast=1)
    always_ff @(posedge clk) begin
        if (st == S_IDLE && s_axis_tvalid && s_axis_tready) begin
            r_req_opcode    <= s_axis_tdata[127:124];
            r_req_addr20    <= s_axis_tdata[123:104];
            r_req_burst_len <= s_axis_tdata[103:88];
            r_req_return_id <= s_axis_tdata[87:80];
        end
    end

    // ---- ddr 读 ----
    // S_FETCH: 拉 re, addr = req_addr20 + data_idx
    assign ddr_re    = (st == S_FETCH);
    assign ddr_raddr = {{(ADDR_W-20){1'b0}}, r_req_addr20} + {{(ADDR_W-16){1'b0}}, r_data_idx};

    // ---- AXIS master (发 RESP) ----
    logic [DATA_W-1:0] resp_header;
    assign resp_header = {4'h2,                     // opcode = READ_RESP
                          r_req_addr20,             // [123:104]
                          r_req_burst_len,          // [103:88]
                          r_req_return_id,          // [87:80] (debug 复制)
                          80'd0};                   // reserved

    always_comb begin
        m_axis_tvalid = 1'b0;
        m_axis_tdata  = '0;
        m_axis_tlast  = 1'b0;
        m_axis_tdest  = r_req_return_id;
        case (st)
            S_HEADER: begin
                m_axis_tvalid = 1'b1;
                m_axis_tdata  = resp_header;
                m_axis_tlast  = 1'b0;       // 后面还有 data flit
            end
            S_DATA: begin
                m_axis_tvalid = 1'b1;
                m_axis_tdata  = ddr_rdata;
                m_axis_tlast  = (r_data_idx == r_req_burst_len - 16'd1);
            end
            default: ;
        endcase
    end

    // ---- FSM ----
    state_t st_next;
    always_comb begin
        st_next = st;
        case (st)
            S_IDLE  : if (s_axis_tvalid && s_axis_tready) st_next = S_HEADER;
            S_HEADER: if (m_axis_tvalid && m_axis_tready) st_next = S_FETCH;
            S_FETCH : st_next = S_DATA;     // 1 拍后 ddr_rdata 有效
            S_DATA  : if (m_axis_tvalid && m_axis_tready) begin
                if (r_data_idx == r_req_burst_len - 16'd1) st_next = S_IDLE;
                else                                        st_next = S_FETCH;
            end
            default : st_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) st <= S_IDLE;
        else        st <= st_next;
    end

    // r_data_idx 更新
    always_ff @(posedge clk) begin
        if      (!rst_n)                                     r_data_idx <= '0;
        else if (st == S_HEADER && m_axis_tvalid && m_axis_tready) r_data_idx <= '0;
        else if (st == S_DATA   && m_axis_tvalid && m_axis_tready) r_data_idx <= r_data_idx + 16'd1;
    end

endmodule
