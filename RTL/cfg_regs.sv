`timescale 1ns/1ps

// =============================================================================
// cfg_regs.sv  --  Shared Configuration Register Bank
//
// 集中持有所有层配置参数，对外以只读端口暴露给 5 个计算/搬运模块
// (act_feeder / wgt_feeder / mac_array / parf_accum / ofb_writer)。
//
// TB 通过层次引用直接写入内部寄存器（无外部写端口）：
//     u_core_top.u_cfg.h_out = ...;
//
// 字段语义见 docs/config-registers.md。
// =============================================================================

module cfg_regs #(
    parameter int ADDR_W = 20
)(
    input  logic                clk,
    input  logic                rst_n,

    // Loop bounds
    output logic [15:0]         h_out,
    output logic [15:0]         w_out,
    output logic [15:0]         w_in,
    output logic [3:0]          k,
    output logic [2:0]          stride,
    output logic [5:0]          cin_slices,
    output logic [5:0]          cout_slices,
    output logic [5:0]          tile_w,
    output logic [7:0]          num_tiles,
    output logic [5:0]          last_valid_w,

    // Weight scheduling
    output logic [9:0]          total_wrf,
    output logic                wrf_packed,
    output logic [9:0]          kk,
    output logic [2:0]          rounds_per_cins,
    output logic [5:0]          round_len_last,

    // Address bases (20-bit)
    output logic [ADDR_W-1:0]   ifb_base,
    output logic [ADDR_W-1:0]   wb_base,
    output logic [ADDR_W-1:0]   ofb_base,

    // Address steps (20-bit)
    output logic [ADDR_W-1:0]   ifb_cin_step,
    output logic [ADDR_W-1:0]   ifb_row_step,
    output logic [ADDR_W-1:0]   wb_cin_step,
    output logic [ADDR_W-1:0]   wb_cout_step,
    output logic [ADDR_W-1:0]   ofb_cout_step,
    output logic [ADDR_W-1:0]   tile_in_step,

    // SDP
    output logic [4:0]          sdp_shift,
    output logic                sdp_relu_en
);

    // ---- 内部寄存器（TB 层次引用写入） ----
    logic [15:0]        r_h_out = 16'd0;
    logic [15:0]        r_w_out = 16'd0;
    logic [15:0]        r_w_in  = 16'd0;
    logic [3:0]         r_k     = 4'd0;
    logic [2:0]         r_stride = 3'd0;
    logic [5:0]         r_cin_slices  = 6'd0;
    logic [5:0]         r_cout_slices = 6'd0;
    logic [5:0]         r_tile_w      = 6'd0;
    logic [7:0]         r_num_tiles   = 8'd0;
    logic [5:0]         r_last_valid_w = 6'd0;

    logic [9:0]         r_total_wrf = 10'd0;
    logic               r_wrf_packed = 1'b0;
    logic [9:0]         r_kk = 10'd0;
    logic [2:0]         r_rounds_per_cins = 3'd0;
    logic [5:0]         r_round_len_last  = 6'd0;

    logic [ADDR_W-1:0]  r_ifb_base = '0;
    logic [ADDR_W-1:0]  r_wb_base  = '0;
    logic [ADDR_W-1:0]  r_ofb_base = '0;

    logic [ADDR_W-1:0]  r_ifb_cin_step   = '0;
    logic [ADDR_W-1:0]  r_ifb_row_step   = '0;
    logic [ADDR_W-1:0]  r_wb_cin_step    = '0;
    logic [ADDR_W-1:0]  r_wb_cout_step   = '0;
    logic [ADDR_W-1:0]  r_ofb_cout_step  = '0;
    logic [ADDR_W-1:0]  r_tile_in_step   = '0;

    logic [4:0]         r_sdp_shift   = 5'd0;
    logic               r_sdp_relu_en = 1'b0;

    // ---- 直通到输出端口 ----
    assign h_out           = r_h_out;
    assign w_out           = r_w_out;
    assign w_in            = r_w_in;
    assign k               = r_k;
    assign stride          = r_stride;
    assign cin_slices      = r_cin_slices;
    assign cout_slices     = r_cout_slices;
    assign tile_w          = r_tile_w;
    assign num_tiles       = r_num_tiles;
    assign last_valid_w    = r_last_valid_w;
    assign total_wrf       = r_total_wrf;
    assign wrf_packed      = r_wrf_packed;
    assign kk              = r_kk;
    assign rounds_per_cins = r_rounds_per_cins;
    assign round_len_last  = r_round_len_last;
    assign ifb_base        = r_ifb_base;
    assign wb_base         = r_wb_base;
    assign ofb_base        = r_ofb_base;
    assign ifb_cin_step    = r_ifb_cin_step;
    assign ifb_row_step    = r_ifb_row_step;
    assign wb_cin_step     = r_wb_cin_step;
    assign wb_cout_step    = r_wb_cout_step;
    assign ofb_cout_step   = r_ofb_cout_step;
    assign tile_in_step    = r_tile_in_step;
    assign sdp_shift       = r_sdp_shift;
    assign sdp_relu_en     = r_sdp_relu_en;

endmodule
