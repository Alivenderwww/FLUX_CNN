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
    parameter int SRAM_DEPTH = 1024,        // OFB 深度 (core_top 实例化按 FLUX_OFB_DEPTH override)
    parameter int ADDR_W     = 20,
    // SB_ADDR_W = $clog2(SHORTCUT_DEPTH); 默认跟 SRAM_DEPTH 共用 (向后兼容).
    // core_top 实例化时显式传 SHB 独立宽, 让 IFB / SHB 物理深度可不同.
    parameter int SB_ADDR_W  = $clog2(SRAM_DEPTH)
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

    // ---- Streaming 配置 (J-2 起恒启用) ----
    // OFB 永远是 ring; ofb_ptr 按 cfg_ofb_ring_words wrap;
    // (rows_written - rows_drained >= cfg_ofb_strip_rows) 时反压 acc_out_ready=0。
    // 整图装得下时 strip=H_OUT ring_words 覆盖整图, wrap 不触发 (退化 batch)。
    input  logic [ADDR_W-1:0]                    cfg_ofb_ring_words,  // 环大小 in words
    input  logic [ADDR_W-1:0]                    cfg_ofb_row_words,   // W_OUT × cout_slices (R.2 shortcut 地址用)
    input  logic [15:0]                          cfg_ofb_strip_rows,  // 6→8→16 bit (任意 H fit)
    input  logic [15:0]                          rows_drained,       // 来自 ODMA

    // ---- R.1: SDP residual / shortcut / bias cfg ----
    input  logic                                 cfg_residual_en,
    input  logic signed [15:0]                   cfg_shortcut_mult,
    input  logic [4:0]                           cfg_shortcut_shift,

    // ---- R.1: bias 来自 bias_rf, shortcut 来自 Shortcut Bank ----
    input  logic [NUM_COL*PSUM_WIDTH-1:0]        bias_in,            // bias_rf 输出
    input  logic                                 bias_ready,         // bias_rf 当前 cs 已就绪
    output logic [5:0]                           cs_cnt_out,         // 给 bias_rf 跟踪
    output logic                                 sb_re,              // Shortcut Bank 读使能
    output logic [SB_ADDR_W-1:0]                 sb_raddr,           // Shortcut Bank 读地址 (SB_ADDR_W 独立)
    input  logic [NUM_COL*DATA_WIDTH-1:0]        sb_rdata,           // Shortcut Bank 读数据 (1 拍延迟)

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
        S_IDLE  = 2'd0,
        S_RUN   = 2'd1,
        S_FLUSH = 2'd2,    // SDP pipeline drain (J-3): 等最后 in-flight 写完
        S_DONE  = 2'd3
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
    // Ring 反压 (永启): 写满 strip_rows 行未被 ODMA 消费则停
    // J-5 timing fix: ring_full 寄存器化, 打断 r_rows_written → 19-level critical chain
    //   acc_out_ready 现在用 register 输出, 起点不再是 r_rows_written (CARRY4 减法链).
    //   driver 配合 ofb_ring_words 多算 1 行 slack (cfg_ofb_strip_rows 比物理 wrap 边界小 1),
    //   让 ring_full 寄存延迟 1 cycle 多写 ≤1 cell 时不 wrap 覆盖 row 0.
    logic ring_full_d;
    always_ff @(posedge clk) begin
        if (!rst_n) ring_full_d <= 1'b0;
        else        ring_full_d <= ((rows_written - rows_drained) >= cfg_ofb_strip_rows);  // 16-bit 直接比较 (rows_written 16-bit, cfg_ofb_strip_rows 16-bit)
    end

    // R.1: bias_rf 在 cs 切换时 4 拍 prefetch, bias_ready=0 期间不放行 acc_fire
    logic acc_fire;
    assign acc_out_ready = (state == S_RUN) && !ring_full_d && bias_ready;
    assign acc_fire      = acc_out_valid && acc_out_ready;

    // 暴露 cs_cnt 给 bias_rf (driven by 内部 counter, declared 后)
    assign cs_cnt_out = cs_cnt;

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
        evt_start          = ((state == S_IDLE) || (state == S_DONE)) && start;
        evt_fire_x_wrap    = acc_fire && x_is_last;
        evt_fire_tile_wrap = evt_fire_x_wrap    && tile_is_last;
        evt_fire_cs_wrap   = evt_fire_tile_wrap && cs_is_last;
        evt_fire_yout_wrap = evt_fire_cs_wrap   && yout_is_last;
    end

    // =========================================================================
    // R.1+J-3: 2-stage pipeline (Stage 0 → 1 → 2; valid 跟 sideband lockstep)
    //
    // Shortcut Bank SRAM 同步读: 周期 N 发 sb_re, 周期 N+1 sb_rdata 有效.
    // SDP 流水化 (J-3): mult 进 DSP48 input register (1-cycle), 输出在 N+2 拍 valid.
    //
    // Stage 0 (cycle N):   acc_fire → 拉 shortcut (sb_raddr = sb_ptr).
    //                       counters 推进, 算 evt_fire_cs_wrap.
    // Stage 1 (cycle N+1): pipe_valid + pipe_psum/bias/ofb_waddr/evt_cs_wrap latch.
    //                       sb_rdata 到达, SDP 输入 (psum, bias, shortcut, valid).
    //                       SDP 内部 stage1 latch (prod_d, shortcut_d, valid_d).
    // Stage 2 (cycle N+2): SDP.valid_out=1, ofm_data 组合输出.
    //                       pipe2_ofb_waddr / pipe2_evt_cs_wrap 跟 valid_out 同拍 align.
    //                       ofb_we = SDP.valid_out, ofb_wdata = sdp_out, OFB 写.
    //                       row_done_pulse = pipe2_evt_cs_wrap & SDP.valid_out.
    // =========================================================================
    logic                                 pipe_valid;
    logic signed [NUM_COL*PSUM_WIDTH-1:0] pipe_psum;
    logic [NUM_COL*PSUM_WIDTH-1:0]        pipe_bias;
    logic [ADDR_W-1:0]                    pipe_ofb_waddr;
    logic                                 pipe_evt_cs_wrap;

    // Stage 1 → 2 latch (SDP 内部已 latch valid/data, 这里 latch sideband 跟 valid lockstep)
    logic [ADDR_W-1:0]                    pipe2_ofb_waddr;
    logic                                 pipe2_evt_cs_wrap;

    // sb_raddr: Shortcut Bank shortcut 段地址.
    //
    // 注意 OFB SRAM 与 Shortcut Bank 的字布局不同:
    //   OFB    布局 = (yout, cs, x) [compute 顺序, ofb_ptr 简单 +1]
    //   Shortcut 布局 = (yout, x, cs)  [DDR NHWC, rdma 线性写]
    // 因此 shortcut 地址不能用 +1 增量, 必须按 (yout, x_abs, cs) 重新组合算:
    //   sb_raddr = shortcut_base + yout × W_OUT × cs + x_abs × cs + cs_cnt
    // 用现有 counter (yout_cnt / cs_cnt / tile_cnt / x_cnt) 组合得 x_abs.
    logic [12:0] shortcut_section_base;
    assign shortcut_section_base = {{(13-8){1'b0}}, cfg_cout_slices, 2'b0};  // cs × 4 = bias section size

    // sb_x_off / sb_yout_off 全用累加器, 消除 2 个 cascade DSP48 mult (J-3 timing fix):
    //   原 sb_x_off = (tile_cnt*tile_w + x_cnt) * cout_slices, 12 levels CARRY4=5 +
    //   DSP48E1=2 + LUTs, critical path 10.6 ns. 累加器替换后 sb_raddr 计算只剩
    //   2 个 register + 4-input add, 大幅缩短到 ~3-4 levels.
    logic [19:0] sb_yout_off;            // yout × W_OUT × cs, evt_fire_cs_wrap 推进
    logic [15:0] sb_x_off;                // (tile*tile_w + x_cnt) × cs, acc_fire 推进
    logic [15:0] sb_x_off_tile_base;      // tile*tile_w*cs base for current tile

    // yout 偏移: 每完整 yout (= cs_wrap) 推进 cfg_ofb_row_words
    always_ff @(posedge clk) begin
        if      (evt_start)             sb_yout_off <= '0;
        else if (evt_fire_cs_wrap)      sb_yout_off <= sb_yout_off + cfg_ofb_row_words;
    end

    // cfg_tile_w_x_cs = tile_w × cs, 在 layer 内是常量 (cfg 不变).
    //   evt_start 时 latch 一次, 避免每 cycle comb mult 占 critical path.
    logic [15:0] cfg_tile_w_x_cs;
    always_ff @(posedge clk) begin
        if      (!rst_n)     cfg_tile_w_x_cs <= 16'd0;
        else if (evt_start)  cfg_tile_w_x_cs <= {10'd0, cfg_tile_w} * {10'd0, cfg_cout_slices};
    end

    always_ff @(posedge clk) begin
        if      (evt_start)             sb_x_off_tile_base <= 16'd0;
        else if (evt_fire_tile_wrap)    sb_x_off_tile_base <= 16'd0;     // cs 推进, x_abs 重置
        else if (evt_fire_x_wrap)       sb_x_off_tile_base <= sb_x_off_tile_base + cfg_tile_w_x_cs;
    end

    // sb_x_off: tile_base + x_cnt*cs. x_cnt*cs 也累加 (inner-most, 每 acc_fire +=cs)
    //   实际等价: tile 内每 cell sb_x_off += cs; tile_wrap 时跳到 tile_base (新 tile 起点).
    always_ff @(posedge clk) begin
        if      (evt_start)             sb_x_off <= 16'd0;
        else if (evt_fire_tile_wrap)    sb_x_off <= 16'd0;
        else if (evt_fire_x_wrap)       sb_x_off <= sb_x_off_tile_base + cfg_tile_w_x_cs;  // 下个 tile 起点
        else if (acc_fire)              sb_x_off <= sb_x_off + {10'd0, cfg_cout_slices};
    end

    assign sb_re    = acc_fire;
    // sb_raddr 内部按 13-bit 算; 输出端口宽度 = SB_ADDR_W (= $clog2(SHORTCUT_DEPTH))
    // SHB 最大段地址 driver 限 < 2048 → 12-bit 足以表达, 截高位安全.
    logic [12:0] sb_raddr_full;
    assign sb_raddr_full = shortcut_section_base
                         + sb_yout_off[12:0]
                         + sb_x_off[12:0]
                         + {{(13-6){1'b0}}, cs_cnt};
    assign sb_raddr = sb_raddr_full[SB_ADDR_W-1:0];

    // Stage 0 → 1 latch
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            pipe_valid <= 1'b0;
        end else begin
            pipe_valid <= acc_fire;
        end
    end

    always_ff @(posedge clk) begin
        if (acc_fire) begin
            pipe_psum        <= acc_out_vec;
            pipe_bias        <= bias_in;
            pipe_ofb_waddr   <= ofb_ptr;
            pipe_evt_cs_wrap <= evt_fire_cs_wrap;
        end
    end

    // =========================================================================
    // SDP (组合, 在 Stage 1 用 latched 数据 + 这拍到达的 sb_rdata)
    // =========================================================================
    logic [NUM_COL*DATA_WIDTH-1:0] sdp_out;

    logic sdp_valid_out;
    sdp #(
        .NUM_COL   (NUM_COL),
        .PSUM_WIDTH(PSUM_WIDTH)
    ) u_sdp (
        .clk            (clk),
        .rst_n          (rst_n),
        .shift_amt      (cfg_sdp_shift),
        .mult           (cfg_sdp_mult),
        .zp_out         (cfg_sdp_zp_out),
        .clip_min       (cfg_sdp_clip_min),
        .clip_max       (cfg_sdp_clip_max),
        .round_en       (cfg_sdp_round_en),
        .relu_en        (cfg_sdp_relu_en),
        .residual_en    (cfg_residual_en),
        .shortcut_mult  (cfg_shortcut_mult),
        .shortcut_shift (cfg_shortcut_shift),
        .psum_in        (pipe_psum),
        .bias_in        (pipe_bias),
        .shortcut_in    (sb_rdata),
        .valid_in       (pipe_valid),
        .ofm_data       (sdp_out),
        .valid_out      (sdp_valid_out)
    );

    // =========================================================================
    // Sideband pipeline (跟 SDP 内部 valid_d → valid_dd → valid_out lockstep)
    //   SDP J-4 内部 3-stage (input → stage1 → stage2a latch → stage2b out),
    //   即 pipe_valid 后过 2 拍 sdp_valid_out 才 high. sideband 也加 2 级 latch:
    //     pipe   (Stage 0→1, 已有)
    //     pipe2  (Stage 1→2 latch, 跟 SDP.valid_d 同拍)
    //     pipe3  (Stage 2→2b latch, 跟 SDP.valid_dd / valid_out 同拍)
    //   valid=0 时 sideband 不被消费 (ofb_we=0), 不需要 reset.
    // =========================================================================
    logic [ADDR_W-1:0]  pipe3_ofb_waddr;
    logic               pipe3_evt_cs_wrap;
    always_ff @(posedge clk) begin
        pipe2_ofb_waddr   <= pipe_ofb_waddr;
        pipe2_evt_cs_wrap <= pipe_evt_cs_wrap;
        pipe3_ofb_waddr   <= pipe2_ofb_waddr;
        pipe3_evt_cs_wrap <= pipe2_evt_cs_wrap;
    end

    // =========================================================================
    // OFB 写 (Stage 2b 出, 用 SDP.valid_out + pipe3_* sideband)
    // =========================================================================
    assign ofb_we    = sdp_valid_out;
    assign ofb_waddr = pipe3_ofb_waddr[AW-1:0];
    assign ofb_wdata = sdp_out;

    // =========================================================================
    // 三段式 FSM
    // =========================================================================
    // SDP pipeline (J-4): SDP 内部 3-stage (input → stage1 latch → stage2a latch → stage2b out).
    // ofb_writer 自身 1-stage (Stage 0→1 latch) + SDP 2-stage = 总 3 stage, max in_flight=3.
    // in_flight 跟踪 Stage 0..2b 内 valid 数, 让 S_FLUSH 等 drain.
    //   acc_fire (Stage 0 入)   → in_flight++
    //   sdp_valid_out (Stage 2b 出) → in_flight--
    logic [2:0] in_flight;
    always_ff @(posedge clk) begin
        if (!rst_n || evt_start) in_flight <= 3'd0;
        else begin
            case ({acc_fire, sdp_valid_out})
                2'b10: in_flight <= in_flight + 3'd1;
                2'b01: in_flight <= in_flight - 3'd1;
                default: in_flight <= in_flight;
            endcase
        end
    end

    // Seg 1: state_next 组合
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (start)                  state_next = S_RUN;
            S_RUN  : if (acc_fire && all_done)   state_next = S_FLUSH;
            S_FLUSH: if (in_flight == 3'd0)      state_next = S_DONE;
            S_DONE : if (start)                  state_next = S_RUN;   // 多 strip / 多 case 重启
            default:                              state_next = S_IDLE;
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
            if (ofb_ptr == ofb_ptr_wrap_limit_m1) ofb_ptr <= cfg_ofb_base;
            else                                  ofb_ptr <= ofb_ptr + {{(ADDR_W-1){1'b0}}, 1'b1};
        end
        else                ofb_ptr <= ofb_ptr;
    end

    // =========================================================================
    // Row-level credit 输出（方式 1, R.1 后延 1 拍跟 OFB 写对齐）
    //   row_done_pulse: pipe_evt_cs_wrap (= evt_fire_cs_wrap 延 1 拍, 跟 ofb_we
    //                   写下一行第一个 word 同拍, 此时该 yout 最后一个 word
    //                   已经写到 OFB SRAM)
    //   rows_written:   累计 yout 数（控制路径，复位必须）
    // =========================================================================
    assign row_done_pulse = pipe3_evt_cs_wrap && sdp_valid_out;

    logic [15:0] r_rows_written;
    always_ff @(posedge clk) begin
        if      (!rst_n)                                 r_rows_written <= 16'd0;
        else if (evt_start)                              r_rows_written <= 16'd0;
        else if (pipe3_evt_cs_wrap && sdp_valid_out)     r_rows_written <= r_rows_written + 16'd1;
        else                                             r_rows_written <= r_rows_written;
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
