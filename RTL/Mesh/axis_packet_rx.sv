`timescale 1ns/1ps

// =============================================================================
// axis_packet_rx.sv  --  AXI4-Stream slave → 多个 SRAM 写口分发
//
// 收到 packet (来自 mesh LOCAL port), 解 header 后写到对应 SRAM:
//   header flit: opcode + addr_offset + burst_len
//   body flit:  data
//   tail flit (tlast=1): 最后 1 个 data flit
//
// 每条 packet 写到 1 个 SRAM 区域 (IFB / WB / SB / RDMA), 由 opcode 决定.
// 内部按 SRAM 输出: sram_we / sram_waddr / sram_wdata, 上层 mux 到具体 SRAM.
//
// 状态机: IDLE → HEADER 接收 → BODY 接 N 个 word 写 SRAM → TAIL → IDLE
// =============================================================================

module axis_packet_rx #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ADDR_W = 20      // SRAM 字偏移
)(
    input  logic                clk,
    input  logic                rst_n,

    // AXIS slave (从 mesh router LOCAL master 来)
    input  logic                s_axis_tvalid,
    output logic                s_axis_tready,
    input  logic [DATA_W-1:0]   s_axis_tdata,
    input  logic                s_axis_tlast,
    input  logic [DEST_W-1:0]   s_axis_tdest,    // 此处不用 (已经路由到 LOCAL 了)

    // 解码后的 SRAM 写口 (上层 mux 到 IFB/WB/SB/RDMA SRAM)
    output logic                sram_we,
    output logic [3:0]          sram_target,     // opcode (WRITE_IFB/WB/SB/RDMA/...)
    output logic [ADDR_W-1:0]   sram_waddr,
    output logic [DATA_W-1:0]   sram_wdata,

    // 状态: 当前 packet 完成 (rising edge on tlast fire)
    output logic                packet_done,
    output logic [3:0]          last_opcode,     // 最近完成 packet 的 opcode
    output logic [15:0]         last_burst_len,
    output logic [ADDR_W-1:0]   last_addr_offset
);

    typedef enum logic [1:0] {S_IDLE, S_BODY} state_t;
    state_t st;

    // 锁存 header 字段
    logic [3:0]          r_opcode;
    logic [ADDR_W-1:0]   r_addr_base;
    logic [15:0]         r_burst_len;
    logic [15:0]         r_body_cnt;       // 已收 body flit 数

    assign s_axis_tready = 1'b1;          // 简化: 总是 ready (上层 SRAM 总能接受)

    // last_* 直接组合输出 r_* (BODY 状态 r_* 是当前 packet 的值, 跟 packet_done 同拍)
    assign last_opcode      = r_opcode;
    assign last_burst_len   = r_burst_len;
    assign last_addr_offset = r_addr_base;

    // header 解析 (在 IDLE 状态收到 valid 时)
    // packet header.addr 是 20-bit (data[123:104]), zero-extend 到 ADDR_W
    logic [3:0]          h_opcode;
    logic [ADDR_W-1:0]   h_addr;
    logic [15:0]         h_burst_len;
    logic [19:0]         h_addr20;
    assign h_opcode    = s_axis_tdata[127:124];
    assign h_addr20    = s_axis_tdata[123:104];
    assign h_addr      = (ADDR_W >= 20) ? {{(ADDR_W-20){1'b0}}, h_addr20}
                                        : h_addr20[ADDR_W-1:0];
    assign h_burst_len = s_axis_tdata[103:88];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st              <= S_IDLE;
            r_opcode        <= '0;
            r_addr_base     <= '0;
            r_burst_len     <= '0;
            r_body_cnt      <= '0;
        end else begin
            case (st)
                S_IDLE: if (s_axis_tvalid && s_axis_tready) begin
                    r_opcode    <= h_opcode;
                    r_addr_base <= h_addr;
                    r_burst_len <= h_burst_len;
                    r_body_cnt  <= '0;
                    if (s_axis_tlast || h_burst_len == 16'd0) begin
                        st <= S_IDLE;
                    end else begin
                        st <= S_BODY;
                    end
                end
                S_BODY: if (s_axis_tvalid && s_axis_tready) begin
                    r_body_cnt <= r_body_cnt + 16'd1;
                    if (s_axis_tlast) begin
                        st <= S_IDLE;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    // SRAM 写口
    assign sram_we     = (st == S_BODY) && s_axis_tvalid && s_axis_tready;
    assign sram_target = r_opcode;
    assign sram_waddr  = r_addr_base + ADDR_W'(r_body_cnt);
    assign sram_wdata  = s_axis_tdata;

    // packet_done: tlast 在 BODY 状态触发, 或 IDLE 状态单 flit packet
    assign packet_done = (st == S_BODY) && s_axis_tvalid && s_axis_tready && s_axis_tlast;

endmodule
