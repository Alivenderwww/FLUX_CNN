`timescale 1ns/1ps

// =============================================================================
// parf_col.sv  --  单列 PSUM 存储
//
// parf_accum 的子模块: 每列独立存一条 PARF_DEPTH 深 × PSUM_WIDTH 宽 的累加器,
// 独立的读写地址 / 写使能. 为 Kx-fold 做准备 — 每列可被指派不同 wr_addr / we
// 以实现按列组偏移写 psum.
//
// 数据路径无复位 (§6): 上游 we 未拉高前读出值被 acc_out_valid 遮蔽.
// =============================================================================
module parf_col #(
    parameter int PSUM_WIDTH = 32,
    parameter int PARF_DEPTH = 32
)(
    input  logic                          clk,

    // 写端口
    input  logic                          we,
    input  logic [$clog2(PARF_DEPTH)-1:0] wr_addr,
    input  logic signed [PSUM_WIDTH-1:0]  wdata,

    // 读端口 (组合读, 给 drain 用)
    input  logic [$clog2(PARF_DEPTH)-1:0] rd_addr,
    output logic signed [PSUM_WIDTH-1:0]  rdata,

    // 当前 wr_addr 的旧值 (给 mac_array acc_seed 融合用)
    output logic signed [PSUM_WIDTH-1:0]  old_at_wr
);

    logic signed [PSUM_WIDTH-1:0] mem [0:PARF_DEPTH-1];

    always_ff @(posedge clk) begin
        if (we) mem[wr_addr] <= wdata;
    end

    assign rdata     = mem[rd_addr];
    assign old_at_wr = mem[wr_addr];

endmodule
