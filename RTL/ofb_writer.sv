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
// 四层循环（方式 1，cs 下移到 yout 内）：
//     for yout in 0..h_out-1
//       for cs in 0..cout_slices-1
//         for tile in 0..num_tiles-1
//           for x in 0..cur_valid_w-1         // cur_valid_w = tile_w 或 last_valid_w
//             一拍 parf valid & ready → SDP → OFB 写 1 addr
//
// ofb_ptr 从 cfg_ofb_base 每 fire 线性 +1，扫完一个 (yout,cs) 写 W_OUT word，
// 扫完一个 yout 写 W_OUT × cout_slices word（NHWC 一行全部 cs 段）。
// streaming 下 ofb_ptr 按 cfg_ofb_ring_words wrap。
// row_done_pulse 只在 evt_fire_cs_wrap (一个 yout 全部 cs 段写完) 时发。
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
    input  logic [5:0]                           cfg_sdp_shift,
    input  logic                                 cfg_sdp_relu_en,
    input  logic signed [31:0]                   cfg_sdp_mult,
    input  logic signed [8:0]                    cfg_sdp_zp_out,
    input  logic signed [8:0]                    cfg_sdp_clip_min,
    input  logic signed [8:0]                    cfg_sdp_clip_max,
    input  logic                                 cfg_sdp_round_en,

    // ---- Streaming 配置（v2） ----
    // cfg_odma_streaming=1 时 OFB 变 ring；ofb_ptr 按 cfg_ofb_ring_words wrap；
    // (rows_written - rows_drained >= cfg_ofb_strip_rows) 时反压 acc_out_ready=0
    input  logic                                 cfg_odma_streaming,
    input  logic [ADDR_W-1:0]                    cfg_ofb_ring_words,  // 环大小 in words
    input  logic [5:0]                           cfg_ofb_strip_rows,
    input  logic [15:0]                          rows_drained,       // 来自 ODMA

    // ---- upstream: parf_accum ----
    input  logic                                 acc_out_valid,
    input  logic signed [NUM_COL*PSUM_WIDTH-1:0] acc_out_vec,
    output logic                                 acc_out_ready,

    // ---- OFB SRAM 写端口 ----
    output logic                                 ofb_we,
    output logic [$clog2(SRAM_DEPTH)-1:0]        ofb_waddr,
    output logic [NUM_COL*DATA_WIDTH-1:0]        ofb_wdata,

    // ---- Row-level credit 输出（给 ODMA streaming mode） ----
    output logic                                 row_done_pulse,   // 每写完 1 行 OFM 脉冲
    output logic [15:0]                          rows_written      // 累计写完的 OFM 行数
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
    // 四层计数器 + running pointer (声明)
    // =========================================================================
    logic [5:0]         x_cnt;        // 0..cur_valid_w-1
    logic [7:0]         tile_cnt;     // 0..cfg_num_tiles-1
    logic [15:0]        yout_cnt;     // 0..cfg_h_out-1
    logic [5:0]         cs_cnt;       // 0..cfg_cout_slices-1
    logic [ADDR_W-1:0]  ofb_ptr;

    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    // =========================================================================
    // 握手 + 边界
    // =========================================================================
    // Streaming ring 反压：写满 strip_rows 行未被 ODMA 消费则停
    logic ring_full;
    assign ring_full = cfg_odma_streaming &&
                       ((rows_written - rows_drained) >= {10'd0, cfg_ofb_strip_rows});

    logic acc_fire;
    assign acc_out_ready = (state == S_RUN) && !ring_full;
    assign acc_fire      = acc_out_valid && acc_out_ready;

    logic x_is_last, tile_is_last, yout_is_last, cs_is_last;
    assign x_is_last    = (x_cnt    == cur_valid_w       - 6'd1);
    assign tile_is_last = (tile_cnt == cfg_num_tiles     - 8'd1);
    assign yout_is_last = (yout_cnt == cfg_h_out         - 16'd1);
    assign cs_is_last   = (cs_cnt   == cfg_cout_slices   - 6'd1);

    logic all_done;
    assign all_done = x_is_last && tile_is_last && yout_is_last && cs_is_last;

    // =========================================================================
    // 命名事件信号（方式 1：cs 下移到 yout 内）
    //   evt_fire_x_wrap   : x 到末尾（tile 推进）
    //   evt_fire_tile_wrap: tile 到末尾（cs 推进 —— 新：一个 (yout,cs) 完成）
    //   evt_fire_cs_wrap  : cs 到末尾（yout 推进 —— 新：一个 yout 全 cs 完成）
    //   evt_fire_yout_wrap: yout 到末尾（整 strip 结束）
    // =========================================================================
    logic evt_start;
    logic evt_fire_x_wrap;
    logic evt_fire_tile_wrap;
    logic evt_fire_cs_wrap;
    logic evt_fire_yout_wrap;

    always_comb begin
        // F-2: start 无条件 evt_start
        evt_start          = start;
        evt_fire_x_wrap    = acc_fire && x_is_last;
        evt_fire_tile_wrap = evt_fire_x_wrap    && tile_is_last;
        evt_fire_cs_wrap   = evt_fire_tile_wrap && cs_is_last;
        evt_fire_yout_wrap = evt_fire_cs_wrap   && yout_is_last;
    end

    // =========================================================================
    // SDP (组合)
    // =========================================================================
    logic [NUM_COL*DATA_WIDTH-1:0] sdp_out;

    sdp #(
        .NUM_COL   (NUM_COL),
        .PSUM_WIDTH(PSUM_WIDTH)
    ) u_sdp (
        .shift_amt (cfg_sdp_shift),
        .mult      (cfg_sdp_mult),
        .zp_out    (cfg_sdp_zp_out),
        .clip_min  (cfg_sdp_clip_min),
        .clip_max  (cfg_sdp_clip_max),
        .round_en  (cfg_sdp_round_en),
        .relu_en   (cfg_sdp_relu_en),
        .psum_in   (acc_out_vec),
        .valid_in  (acc_fire),
        .ofm_data  (sdp_out),
        .valid_out ()
    );

    // =========================================================================
    // OFB 写（同拍 fire）
    // =========================================================================
    assign ofb_we    = acc_fire;
    assign ofb_waddr = ofb_ptr[AW-1:0];
    assign ofb_wdata = sdp_out;

    // =========================================================================
    // 三段式 FSM
    // =========================================================================
    // Seg 1: state_next 组合
    always_comb begin
        state_next = state;
        // F-2: start 从任何状态强制进 S_RUN
        if (start) state_next = S_RUN;
        else case (state)
            S_IDLE : ;   // wait for start
            S_RUN  : if (acc_fire && all_done)   state_next = S_DONE;
            S_DONE : ;
            default: state_next = S_IDLE;
        endcase
    end

    // Seg 2: state 寄存器（独立，控制路径，同步复位）
    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    // Seg 3: Moore 输出（assign done）
    // F-2 多 case：start 同拍 done 立即掉 0（避免 Sequencer 把上一 case 的残留 done 当真）
    assign done = (state == S_DONE) && !start;

    // =========================================================================
    // 计数器 & 指针
    //   state=S_IDLE 期间 acc_fire=0、ofb_we=0，counter/ptr 的上电 X 不产生可
    //   观测副作用；evt_start 到来时一次性初始化。按 §6 不加复位。
    // =========================================================================
    always_ff @(posedge clk) begin
        if      (evt_start)          x_cnt <= '0;
        else if (evt_fire_x_wrap)    x_cnt <= '0;
        else if (acc_fire)           x_cnt <= x_cnt + 6'd1;
        else                         x_cnt <= x_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)           tile_cnt <= '0;
        else if (evt_fire_tile_wrap)  tile_cnt <= '0;
        else if (evt_fire_x_wrap)     tile_cnt <= tile_cnt + 8'd1;
        else                          tile_cnt <= tile_cnt;
    end

    // 方式 1：cs 和 yout 推进事件互换
    always_ff @(posedge clk) begin
        if      (evt_start)           cs_cnt <= '0;
        else if (evt_fire_cs_wrap)    cs_cnt <= '0;
        else if (evt_fire_tile_wrap)  cs_cnt <= cs_cnt + 6'd1;
        else                          cs_cnt <= cs_cnt;
    end

    always_ff @(posedge clk) begin
        if      (evt_start)           yout_cnt <= '0;
        else if (evt_fire_yout_wrap)  yout_cnt <= '0;
        else if (evt_fire_cs_wrap)    yout_cnt <= yout_cnt + 16'd1;
        else                          yout_cnt <= yout_cnt;
    end

    // ofb_ptr：streaming 模式按 cfg_ofb_ring_words wrap 回 cfg_ofb_base
    logic [ADDR_W-1:0] ofb_ptr_wrap_limit_m1;
    assign ofb_ptr_wrap_limit_m1 = cfg_ofb_base + cfg_ofb_ring_words - {{(ADDR_W-1){1'b0}}, 1'b1};

    always_ff @(posedge clk) begin
        if      (evt_start) ofb_ptr <= cfg_ofb_base;
        else if (acc_fire) begin
            if (cfg_odma_streaming && (ofb_ptr == ofb_ptr_wrap_limit_m1))
                ofb_ptr <= cfg_ofb_base;
            else
                ofb_ptr <= ofb_ptr + {{(ADDR_W-1){1'b0}}, 1'b1};
        end
        else                ofb_ptr <= ofb_ptr;
    end

    // =========================================================================
    // Row-level credit 输出（方式 1）
    //   row_done_pulse: 在 evt_fire_cs_wrap（一个 yout 所有 cs 段都写完）发
    //   rows_written:   累计 yout 数（控制路径，复位必须）
    // =========================================================================
    assign row_done_pulse = evt_fire_cs_wrap;

    logic [15:0] r_rows_written;
    always_ff @(posedge clk) begin
        if      (!rst_n)              r_rows_written <= 16'd0;
        else if (evt_start)           r_rows_written <= 16'd0;
        else if (evt_fire_cs_wrap)    r_rows_written <= r_rows_written + 16'd1;
        else                          r_rows_written <= r_rows_written;
    end
    assign rows_written = r_rows_written;

    // =========================================================================
    // Simulation-only: acc 握手计数器 (parf_accum → ofb_writer)
    //   三个同组握手 counter，分支结构一致，合维护（§4.1 例外）。
    // =========================================================================
    // synthesis translate_off
    int hs_acc_fire;
    int hs_acc_stall;
    int hs_acc_idle;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            hs_acc_fire  <= 0;
            hs_acc_stall <= 0;
            hs_acc_idle  <= 0;
        end else if ( acc_out_valid &&  acc_out_ready) begin
            hs_acc_fire  <= hs_acc_fire  + 1;
            hs_acc_stall <= hs_acc_stall;
            hs_acc_idle  <= hs_acc_idle;
        end else if ( acc_out_valid && !acc_out_ready) begin
            hs_acc_fire  <= hs_acc_fire;
            hs_acc_stall <= hs_acc_stall + 1;
            hs_acc_idle  <= hs_acc_idle;
        end else if (!acc_out_valid &&  acc_out_ready) begin
            hs_acc_fire  <= hs_acc_fire;
            hs_acc_stall <= hs_acc_stall;
            hs_acc_idle  <= hs_acc_idle  + 1;
        end else begin
            hs_acc_fire  <= hs_acc_fire;
            hs_acc_stall <= hs_acc_stall;
            hs_acc_idle  <= hs_acc_idle;
        end
    end
    // synthesis translate_on

endmodule
