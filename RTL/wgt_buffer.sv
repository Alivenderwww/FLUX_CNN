`timescale 1ns/1ps

// =============================================================================
// wgt_buffer.sv  --  Weight Stream Feeder (WB → WRF → MAC)
//
// 替代原 core_ctrl 中 LOAD_WGT 阶段 + COMPUTE 阶段的权重驱动逻辑，
// 自带 7 层循环（6 外层 + x 内层）+ LOAD 阶段。
//
// v1: 仅支持 packed（K² × cin_slices ≤ WRF_DEPTH = 32）
//     每 cs 入口 LOAD 一次 total_wrf 个权重，之后 COMPUTE 整个 cs。
//     chunked 支持 (K=7 等) v2 再加。
//
// 循环 (packed):
//     for cs  in 0..cout_slices-1
//       [S_LOAD]  total_wrf 拍 WB 读 + 1 拍 + 写 WRF (延迟 1 拍对齐)
//       [S_COMPUTE] for yout, tile, cins, ky, kx, x
//                   wrf_raddr = wrf_base_kx = cins*KK + ky*K + kx
//                   wgt_valid=1, 按 mac fire 推进
//
// 地址：cur_wb_base_cs 和 wrf_base_* 都用 running 加法推进，零乘法。
// =============================================================================

module wgt_buffer #(
    parameter int NUM_COL    = 16,
    parameter int NUM_PE     = 16,
    parameter int DATA_WIDTH = 8,
    parameter int WRF_DEPTH  = 32,
    parameter int SRAM_DEPTH = 8192,
    parameter int ADDR_W     = 20
)(
    input  logic                                 clk,
    input  logic                                 rst_n,
    input  logic                                 start,
    output logic                                 done,

    // ---- cfg ----
    input  logic [15:0]                          cfg_h_out,
    input  logic [5:0]                           cfg_cin_slices,
    input  logic [5:0]                           cfg_cout_slices,
    input  logic [5:0]                           cfg_tile_w,
    input  logic [7:0]                           cfg_num_tiles,
    input  logic [5:0]                           cfg_last_valid_w,
    input  logic [3:0]                           cfg_k,
    input  logic [9:0]                           cfg_kk,
    input  logic [9:0]                           cfg_total_wrf,
    input  logic [ADDR_W-1:0]                    cfg_wb_base,
    input  logic [ADDR_W-1:0]                    cfg_wb_cout_step,

    // ---- WB 读 ----
    output logic                                 wb_re,
    output logic [$clog2(SRAM_DEPTH)-1:0]        wb_raddr,
    input  logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_rdata,

    // ---- mac_array WRF 写 ----
    output logic [NUM_COL*NUM_PE-1:0]            wrf_we,
    output logic [$clog2(WRF_DEPTH)-1:0]         wrf_waddr,
    output logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wrf_wdata,

    // ---- mac_array COMPUTE ----
    output logic                                 wgt_valid,
    output logic [$clog2(WRF_DEPTH)-1:0]         wrf_raddr,
    input  logic                                 wgt_ready       // = mac.compute_en
);

    localparam int AW  = $clog2(SRAM_DEPTH);
    localparam int WAW = $clog2(WRF_DEPTH);

    // =========================================================================
    // State
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE    = 2'd0,
        S_LOAD    = 2'd1,
        S_COMPUTE = 2'd2,
        S_DONE    = 2'd3
    } state_t;
    state_t state, state_next;

    // =========================================================================
    // 计数器
    // =========================================================================
    logic [5:0]  x_cnt;
    logic [3:0]  kx_cnt;
    logic [3:0]  ky_cnt;
    logic [5:0]  cins_cnt;
    logic [7:0]  tile_cnt;
    logic [15:0] yout_cnt;
    logic [5:0]  cs_cnt;

    // LOAD 阶段
    logic [9:0]  wb_rd_cnt;
    logic [9:0]  wrf_wr_cnt;
    logic        wb_re_d1;

    // Running bases
    logic [ADDR_W-1:0] cur_wb_base_cs;
    logic [5:0]        wrf_base_cins;  // 6-bit 安全域（cfg_kk 可能 >= 32 的临时扩展）
    logic [5:0]        wrf_base_ky;
    logic [5:0]        wrf_base_kx;

    // =========================================================================
    // cur_valid_w / 边界
    // =========================================================================
    logic [5:0] cur_valid_w;
    assign cur_valid_w = (tile_cnt == cfg_num_tiles - 8'd1) ? cfg_last_valid_w : cfg_tile_w;

    logic x_is_last, kx_is_last, ky_is_last, cins_is_last, tile_is_last, yout_is_last, cs_is_last;
    assign x_is_last    = (x_cnt    == cur_valid_w       - 6'd1);
    assign kx_is_last   = (kx_cnt   == cfg_k             - 4'd1);
    assign ky_is_last   = (ky_cnt   == cfg_k             - 4'd1);
    assign cins_is_last = (cins_cnt == cfg_cin_slices    - 6'd1);
    assign tile_is_last = (tile_cnt == cfg_num_tiles     - 8'd1);
    assign yout_is_last = (yout_cnt == cfg_h_out         - 16'd1);
    assign cs_is_last   = (cs_cnt   == cfg_cout_slices   - 6'd1);

    logic cs_compute_last;
    assign cs_compute_last = x_is_last && kx_is_last && ky_is_last && cins_is_last && tile_is_last && yout_is_last;

    // =========================================================================
    // LOAD 输出
    // =========================================================================
    logic load_wb_re_gate;
    assign load_wb_re_gate = (state == S_LOAD) && (wb_rd_cnt < cfg_total_wrf);
    assign wb_re    = load_wb_re_gate;
    assign wb_raddr = cur_wb_base_cs + {{(ADDR_W-10){1'b0}}, wb_rd_cnt};

    assign wrf_we    = {(NUM_COL*NUM_PE){(state == S_LOAD) && wb_re_d1}};
    assign wrf_waddr = wrf_wr_cnt[WAW-1:0];
    assign wrf_wdata = wb_rdata;

    // =========================================================================
    // COMPUTE 输出
    // =========================================================================
    assign wgt_valid = (state == S_COMPUTE);
    assign wrf_raddr = wrf_base_kx[WAW-1:0];

    logic wgt_fire;
    assign wgt_fire = (state == S_COMPUTE) && wgt_valid && wgt_ready;

    // =========================================================================
    // 状态转移
    // =========================================================================
    logic load_done;
    assign load_done = (wrf_wr_cnt == cfg_total_wrf);

    always_comb begin
        state_next = state;
        case (state)
            S_IDLE:    if (start)     state_next = S_LOAD;
            S_LOAD:    if (load_done) state_next = S_COMPUTE;
            S_COMPUTE: if (wgt_fire && cs_compute_last)
                         state_next = cs_is_last ? S_DONE : S_LOAD;
            S_DONE:    ;
            default:   state_next = S_IDLE;
        endcase
    end

    assign done = (state == S_DONE);

    // =========================================================================
    // 主时序
    // =========================================================================
    logic [5:0] kk_lo6;
    logic [5:0] k_lo6;
    assign kk_lo6 = cfg_kk[5:0];            // packed 下 kk ≤ 32 可装 6-bit
    assign k_lo6  = {2'd0, cfg_k};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            x_cnt          <= '0;
            kx_cnt         <= '0;
            ky_cnt         <= '0;
            cins_cnt       <= '0;
            tile_cnt       <= '0;
            yout_cnt       <= '0;
            cs_cnt         <= '0;
            wb_rd_cnt      <= '0;
            wrf_wr_cnt     <= '0;
            wb_re_d1       <= 1'b0;
            cur_wb_base_cs <= '0;
            wrf_base_cins  <= '0;
            wrf_base_ky    <= '0;
            wrf_base_kx    <= '0;
        end else begin
            state    <= state_next;
            wb_re_d1 <= load_wb_re_gate;

            // ---- IDLE: 等 start ----
            if (state == S_IDLE && start) begin
                x_cnt          <= '0;
                kx_cnt         <= '0;
                ky_cnt         <= '0;
                cins_cnt       <= '0;
                tile_cnt       <= '0;
                yout_cnt       <= '0;
                cs_cnt         <= '0;
                wb_rd_cnt      <= '0;
                wrf_wr_cnt     <= '0;
                cur_wb_base_cs <= cfg_wb_base;
                wrf_base_cins  <= '0;
                wrf_base_ky    <= '0;
                wrf_base_kx    <= '0;
            end

            // ---- LOAD: 发 WB 读 + 1 拍后写 WRF ----
            if (state == S_LOAD) begin
                if (load_wb_re_gate) wb_rd_cnt  <= wb_rd_cnt  + 10'd1;
                if (wb_re_d1)        wrf_wr_cnt <= wrf_wr_cnt + 10'd1;
            end

            // ---- LOAD → COMPUTE 切换：重置所有 COMPUTE 计数 ----
            if (state == S_LOAD && state_next == S_COMPUTE) begin
                wb_rd_cnt     <= '0;
                wrf_wr_cnt    <= '0;
                x_cnt         <= '0;
                kx_cnt        <= '0;
                ky_cnt        <= '0;
                cins_cnt      <= '0;
                tile_cnt      <= '0;
                yout_cnt      <= '0;
                wrf_base_cins <= '0;
                wrf_base_ky   <= '0;
                wrf_base_kx   <= '0;
            end

            // ---- COMPUTE: mac fire 推进嵌套 counters + wrf 基址 ----
            if (state == S_COMPUTE && wgt_fire) begin
                if (x_is_last) begin
                    x_cnt <= '0;
                    if (kx_is_last) begin
                        kx_cnt <= '0;
                        if (ky_is_last) begin
                            ky_cnt <= '0;
                            if (cins_is_last) begin
                                cins_cnt <= '0;
                                if (tile_is_last) begin
                                    tile_cnt <= '0;
                                    if (yout_is_last) begin
                                        yout_cnt <= '0;
                                        if (!cs_is_last) begin
                                            cs_cnt         <= cs_cnt + 6'd1;
                                            cur_wb_base_cs <= cur_wb_base_cs + cfg_wb_cout_step;
                                            // wrf_base_* 留到 LOAD→COMPUTE 切换重置
                                        end
                                    end else begin
                                        yout_cnt      <= yout_cnt + 16'd1;
                                        wrf_base_cins <= '0;
                                        wrf_base_ky   <= '0;
                                        wrf_base_kx   <= '0;
                                    end
                                end else begin
                                    tile_cnt      <= tile_cnt + 8'd1;
                                    wrf_base_cins <= '0;
                                    wrf_base_ky   <= '0;
                                    wrf_base_kx   <= '0;
                                end
                            end else begin
                                cins_cnt      <= cins_cnt + 6'd1;
                                wrf_base_cins <= wrf_base_cins + kk_lo6;
                                wrf_base_ky   <= wrf_base_cins + kk_lo6;
                                wrf_base_kx   <= wrf_base_cins + kk_lo6;
                            end
                        end else begin
                            ky_cnt      <= ky_cnt + 4'd1;
                            wrf_base_ky <= wrf_base_ky + k_lo6;
                            wrf_base_kx <= wrf_base_ky + k_lo6;
                        end
                    end else begin
                        kx_cnt      <= kx_cnt + 4'd1;
                        wrf_base_kx <= wrf_base_kx + 6'd1;
                    end
                end else begin
                    x_cnt <= x_cnt + 6'd1;
                end
            end
        end
    end

endmodule
