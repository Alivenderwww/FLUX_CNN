`timescale 1ns/1ps

// =============================================================================
// ofb_writer.sv  --  SDP Post-Process + OFB Writer
//
// 职责：
//   1. 吃 parf_accum 的 acc_out 流（每拍 NUM_COL × PSUM_WIDTH 的累加结果）
//   2. 经 SDP（shift → ReLU → clip 到 uint8）
//   3. 写 OFB（NUM_COL × 8-bit = 128-bit per addr）
//
// 地址推进：OFB 物理连续（ofb_cout_step = h_out × w_out，
//           一行 w_out = sum(tile_widths) 完美覆盖），
//           所以 ofb_ptr 从 cfg_ofb_base 起步，每 fire 一拍就 +1。
//
// 四层循环（自己跑，独立于其他模块）：
//     for cs in 0..cout_slices-1
//       for yout in 0..h_out-1
//         for tile in 0..num_tiles-1
//           for x in 0..cur_valid_w-1         // cur_valid_w = tile_w 或 last_valid_w
//             一拍 parf valid & ready → SDP → OFB 写 1 addr
//
// SDP 是纯组合，和 OFB 写同拍完成（无额外延迟）。
// =============================================================================

module ofb_writer #(
    parameter int NUM_COL    = 16,
    parameter int DATA_WIDTH = 8,
    parameter int PSUM_WIDTH = 32,
    parameter int SRAM_DEPTH = 8192,
    parameter int ADDR_W     = 20
)(
    input  logic                                 clk,
    input  logic                                 rst_n,

    // 启动 / 完成握手
    input  logic                                 start,
    output logic                                 done,

    // ---- cfg ----
    input  logic [15:0]                          cfg_h_out,
    input  logic [5:0]                           cfg_tile_w,
    input  logic [5:0]                           cfg_last_valid_w,
    input  logic [7:0]                           cfg_num_tiles,
    input  logic [5:0]                           cfg_cout_slices,
    input  logic [ADDR_W-1:0]                    cfg_ofb_base,
    input  logic [4:0]                           cfg_sdp_shift,
    input  logic                                 cfg_sdp_relu_en,

    // ---- upstream: parf_accum ----
    input  logic                                 acc_out_valid,
    input  logic signed [NUM_COL*PSUM_WIDTH-1:0] acc_out_vec,
    output logic                                 acc_out_ready,

    // ---- OFB SRAM 写端口 ----
    output logic                                 ofb_we,
    output logic [$clog2(SRAM_DEPTH)-1:0]        ofb_waddr,
    output logic [NUM_COL*DATA_WIDTH-1:0]        ofb_wdata
);

    localparam int AW = $clog2(SRAM_DEPTH);

    // =========================================================================
    // 状态机
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_RUN  = 2'd1,
        S_DONE = 2'd2
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 四层计数器 + running pointer
    // =========================================================================
    logic [5:0]         x_cnt;        // 0..cur_valid_w-1
    logic [7:0]         tile_cnt;     // 0..cfg_num_tiles-1
    logic [15:0]        yout_cnt;     // 0..cfg_h_out-1
    logic [5:0]         cs_cnt;       // 0..cfg_cout_slices-1
    logic [ADDR_W-1:0]  ofb_ptr;

    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // =========================================================================
    // 握手
    // =========================================================================
    logic fire;
    assign acc_out_ready = (state == S_RUN);
    assign fire          = (state == S_RUN) && acc_out_valid && acc_out_ready;

    // =========================================================================
    // 边界
    // =========================================================================
    logic x_is_last, tile_is_last, yout_is_last, cs_is_last;
    assign x_is_last    = (x_cnt    == cur_valid_w       - 6'd1);
    assign tile_is_last = (tile_cnt == cfg_num_tiles     - 8'd1);
    assign yout_is_last = (yout_cnt == cfg_h_out         - 16'd1);
    assign cs_is_last   = (cs_cnt   == cfg_cout_slices   - 6'd1);

    logic all_done;
    assign all_done = x_is_last && tile_is_last && yout_is_last && cs_is_last;

    // =========================================================================
    // SDP (组合)
    // =========================================================================
    logic [NUM_COL*DATA_WIDTH-1:0] sdp_out;

    sdp #(
        .NUM_COL   (NUM_COL),
        .PSUM_WIDTH(PSUM_WIDTH)
    ) u_sdp (
        .shift_amt (cfg_sdp_shift),
        .psum_in   (acc_out_vec),
        .valid_in  (fire),
        .relu_en   (cfg_sdp_relu_en),
        .ofm_data  (sdp_out),
        .valid_out ()
    );

    // =========================================================================
    // OFB 写（同拍 fire）
    // =========================================================================
    assign ofb_we    = fire;
    assign ofb_waddr = ofb_ptr[AW-1:0];
    assign ofb_wdata = sdp_out;

    // =========================================================================
    // 状态转移
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE:  if (start)             state_next = S_RUN;
            S_RUN:   if (fire && all_done)  state_next = S_DONE;
            S_DONE:  ;
            default: state_next = S_IDLE;
        endcase
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // Simulation-only: acc 握手计数器 (parf_accum → ofb_writer)
    // =========================================================================
    // synthesis translate_off
    int hs_acc_fire;
    int hs_acc_stall;
    int hs_acc_idle;
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if      ( acc_out_valid &&  acc_out_ready) hs_acc_fire  <= hs_acc_fire  + 1;
            else if ( acc_out_valid && !acc_out_ready) hs_acc_stall <= hs_acc_stall + 1;
            else if (!acc_out_valid &&  acc_out_ready) hs_acc_idle  <= hs_acc_idle  + 1;
        end else begin
            hs_acc_fire  <= 0;
            hs_acc_stall <= 0;
            hs_acc_idle  <= 0;
        end
    end
    // synthesis translate_on

    // =========================================================================
    // 时序更新
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            x_cnt    <= '0;
            tile_cnt <= '0;
            yout_cnt <= '0;
            cs_cnt   <= '0;
            ofb_ptr  <= '0;
        end else begin
            state <= state_next;

            if (state == S_IDLE && start) begin
                x_cnt    <= '0;
                tile_cnt <= '0;
                yout_cnt <= '0;
                cs_cnt   <= '0;
                ofb_ptr  <= cfg_ofb_base;
            end else if (fire) begin
                // 推进 4 层计数：x → tile → yout → cs
                if (x_is_last) begin
                    x_cnt <= '0;
                    if (tile_is_last) begin
                        tile_cnt <= '0;
                        if (yout_is_last) begin
                            yout_cnt <= '0;
                            if (!cs_is_last) cs_cnt <= cs_cnt + 6'd1;
                            // all_done 时 counters 停在末尾
                        end else begin
                            yout_cnt <= yout_cnt + 16'd1;
                        end
                    end else begin
                        tile_cnt <= tile_cnt + 8'd1;
                    end
                end else begin
                    x_cnt <= x_cnt + 6'd1;
                end

                // 地址连续递增
                ofb_ptr <= ofb_ptr + {{(ADDR_W-1){1'b0}}, 1'b1};
            end
        end
    end

endmodule
