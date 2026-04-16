`timescale 1ns/1ps

// =============================================================================
// core_ctrl.sv  --  Configuration-Register Driven Self-Sequencing Controller
//
// TB 通过层次引用直接写入配置寄存器（无外部写接口），拉 start 后 FSM 自驱动：
//    for cs   in 0..cout_slices-1          (cs_cnt)
//      [packed: LOAD_WGT 一次性灌 total_wrf 权重]
//      for yout in 0..h_out-1              (yout_cnt)
//        for tile in 0..num_tiles-1        (tile_cnt)
//          for cins in 0..cin_slices-1     (cins_cnt)
//            for round in 0..rounds_per_cins-1  (round_cnt)
//              [chunked: LOAD_WGT 每 (cins,round) 灌 cur_round_len 权重]
//              for pos in 0..cur_round_len-1   (pos_in_round, 与全局 ky/kx 同步推进)
//          ST_OFM (valid_w cycles)
//
// 轮次 (rounds) 用于处理 K*K > WRF_DEPTH 的情况（K>=7）：
//   - 每 cins 内的 K*K 个权重按 WRF_DEPTH=32 为边界切片
//   - 每 round 先 LOAD_WGT 再跑 cur_round_len 个 sub_op
//   - 轮次之间 PARF 不清零，ky/kx 继续推进（不重置）
//
// 延迟流水与原 datapath 契约不变：
//   wrf_we_d1  匹配 WB 1 拍读延迟
//   arf_we_d1  匹配 IFB 1 拍读延迟
//   ofb_we_d2  匹配 PARF 内部 2 拍对齐
//
// 地址位宽：cfg_*_base / cfg_*_step / 运行指针均为 ADDR_W=20-bit（可容 1M 字）。
// =============================================================================

module core_ctrl #(
    parameter int NUM_COL     = 16,
    parameter int NUM_PE      = 16,
    parameter int WRF_DEPTH   = 32,
    parameter int ARF_DEPTH   = 32,
    parameter int PARF_DEPTH  = 32,
    parameter int SRAM_DEPTH  = 8192
)(
    input  logic                                clk,
    input  logic                                rst_n,

    input  logic                                start,
    output logic                                done,

    output logic                                wb_re,
    output logic [$clog2(SRAM_DEPTH)-1:0]       wb_raddr,
    output logic                                ifb_re,
    output logic [$clog2(SRAM_DEPTH)-1:0]       ifb_raddr,

    output logic                                ofb_we,
    output logic [$clog2(SRAM_DEPTH)-1:0]       ofb_waddr,
    output logic                                sdp_en_out,

    output logic [NUM_COL*NUM_PE-1:0]           wrf_we,
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_waddr,

    output logic                                arf_we,
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_waddr,
    output logic [$clog2(ARF_DEPTH)-1:0]        arf_read_addr,

    output logic                                compute_en,
    output logic [$clog2(WRF_DEPTH)-1:0]        wrf_raddr,
    output logic [$clog2(PARF_DEPTH)-1:0]       parf_addr,
    output logic                                parf_clear,
    output logic                                parf_we,

    output logic                                sdp_shift_we,
    output logic [4:0]                          sdp_shift_wdata
);

    localparam int AW     = $clog2(SRAM_DEPTH);
    localparam int WAW    = $clog2(WRF_DEPTH);
    localparam int AAW    = $clog2(ARF_DEPTH);
    localparam int PAW    = $clog2(PARF_DEPTH);
    localparam int ADDR_W = 20;  // 内部地址/运行指针位宽；支持 SRAM_DEPTH 最高 1M

    //=========================================================================
    // 1. 配置寄存器  (TB hierarchical poke)
    //=========================================================================
    logic [15:0] cfg_h_out,  cfg_w_out;
    logic [15:0] cfg_w_in;
    logic [3:0]  cfg_k;
    logic [2:0]  cfg_stride;
    logic [5:0]  cfg_cin_slices, cfg_cout_slices;
    logic [5:0]  cfg_tile_w;
    logic [9:0]  cfg_total_wrf;              // packed 模式一次性灌
    logic        cfg_wrf_packed;
    logic [9:0]  cfg_kk;                     // K*K
    logic [2:0]  cfg_rounds_per_cins;        // ceil(kk/WRF_DEPTH); packed/chunked-single=1
    logic [5:0]  cfg_round_len_last;         // 末轮长度 (kk - (rounds-1)*32)
    logic [ADDR_W-1:0] cfg_ifb_base, cfg_wb_base, cfg_ofb_base;
    logic [ADDR_W-1:0] cfg_ifb_cin_step;
    logic [ADDR_W-1:0] cfg_ifb_row_step;
    logic [ADDR_W-1:0] cfg_wb_cin_step;
    logic [ADDR_W-1:0] cfg_wb_cout_step;
    logic [ADDR_W-1:0] cfg_ofb_cout_step;
    logic [7:0]        cfg_num_tiles;
    logic [5:0]        cfg_last_valid_w;
    logic [ADDR_W-1:0] cfg_tile_in_step;     // TILE_W * stride
    logic [4:0]        cfg_sdp_shift;
    logic              cfg_sdp_relu_en;

    //=========================================================================
    // 2. FSM
    //=========================================================================
    typedef enum logic [3:0] {
        S_IDLE,
        S_SHIFT_WR,
        S_LOAD_WGT,
        S_SUBOP,
        S_ST_OFM,
        S_DONE
    } state_t;

    state_t state, nstate;

    //=========================================================================
    // 3. 循环计数器
    //=========================================================================
    logic [5:0]  cs_cnt;
    logic [15:0] yout_cnt;
    logic [7:0]  tile_cnt;
    logic [5:0]  cins_cnt;
    logic [3:0]  ky_cnt, kx_cnt;
    logic [2:0]  round_cnt;                  // 0..rounds_per_cins-1
    logic [5:0]  pos_in_round;                // 0..cur_round_len-1, 也用作 wrf 读偏移
    logic [15:0] sub_cnt;                     // LOAD_WGT / SUBOP / ST_OFM 内部计数
    logic        first_tile_subop;

    //=========================================================================
    // 4. 当前轮次长度（组合）
    //=========================================================================
    logic [5:0] cur_round_len;
    always_comb begin
        if (round_cnt == cfg_rounds_per_cins - 3'd1)
            cur_round_len = cfg_round_len_last;
        else
            cur_round_len = 6'd32;
    end

    //=========================================================================
    // 5. 子 op 参数
    //=========================================================================
    logic [15:0] valid_w;
    always_comb begin
        if (tile_cnt == cfg_num_tiles - 8'd1)
            valid_w = {10'd0, cfg_last_valid_w};
        else
            valid_w = {10'd0, cfg_tile_w};
    end

    logic stride1;
    assign stride1 = (cfg_stride == 3'd0) || (cfg_stride == 3'd1);

    logic full_tile;
    assign full_tile = (valid_w == {10'd0, cfg_tile_w});

    logic sliding;
    assign sliding = stride1 && full_tile;

    logic [5:0]  sub_ld_len;
    logic [15:0] sub_mac_len;
    logic [4:0]  sub_arf_rd_base;
    logic [4:0]  sub_arf_wr_base;
    logic [15:0] sub_ifb_offset;
    logic [2:0]  sub_eff_stride;

    always_comb begin
        if (sliding) begin
            if (kx_cnt == 4'd0) begin
                sub_ld_len      = cfg_tile_w;
                sub_mac_len     = valid_w;
                sub_arf_rd_base = 5'd0;
                sub_arf_wr_base = 5'd0;
                sub_ifb_offset  = 16'd0;
                sub_eff_stride  = 3'd1;
            end else begin
                sub_ld_len      = 6'd1;
                sub_mac_len     = valid_w;
                sub_arf_rd_base = {1'b0, kx_cnt};
                sub_arf_wr_base = {1'b0, kx_cnt - 4'd1};
                sub_ifb_offset  = {10'd0, cfg_tile_w} + {12'd0, kx_cnt} - 16'd1;
                sub_eff_stride  = 3'd1;
            end
        end else begin
            sub_ld_len      = valid_w[5:0];
            sub_mac_len     = valid_w;
            sub_arf_rd_base = 5'd0;
            sub_arf_wr_base = 5'd0;
            sub_ifb_offset  = {12'd0, kx_cnt};
            sub_eff_stride  = stride1 ? 3'd1 : cfg_stride;
        end
    end

    logic [15:0] sub_end_cnt;
    assign sub_end_cnt = sub_mac_len;

    //=========================================================================
    // 6. 运行指针（全加法）
    //=========================================================================
    logic [ADDR_W-1:0] r_ifb_yout;
    logic [ADDR_W-1:0] r_ifb_cins;
    logic [ADDR_W-1:0] r_ifb_ky;
    logic [ADDR_W-1:0] r_ofb_cs;
    logic [ADDR_W-1:0] r_ofb_yout;
    logic [ADDR_W-1:0] r_wb_cs;
    logic [ADDR_W-1:0] r_wb_cins;
    logic [ADDR_W-1:0] x_tile_out_r;
    logic [ADDR_W-1:0] x_tile_in_r;
    logic [WAW-1:0]    r_wrf_base;          // packed: cins*kk; chunked: 0

    //=========================================================================
    // 7. 延迟流水
    //=========================================================================
    logic [NUM_COL*NUM_PE-1:0]  wrf_we_d1;
    logic [WAW-1:0]             wrf_waddr_d1;
    logic                       arf_we_d1;
    logic [AAW-1:0]             arf_waddr_d1;
    logic                       ofb_we_d1, ofb_we_d2;
    logic [AW-1:0]              ofb_waddr_d1, ofb_waddr_d2;
    logic                       sdp_en_d1, sdp_en_d2;

    //=========================================================================
    // 8. 推进条件
    //=========================================================================
    logic subop_last_cycle;
    assign subop_last_cycle = (state == S_SUBOP) && (sub_cnt == sub_end_cnt);

    logic kx_last, ky_last, cins_last, tile_last, yout_last, cs_last;
    logic round_last, pos_last;
    assign kx_last    = (kx_cnt   == cfg_k - 4'd1);
    assign ky_last    = (ky_cnt   == cfg_k - 4'd1);
    assign cins_last  = (cins_cnt == cfg_cin_slices - 6'd1);
    assign tile_last  = (tile_cnt == cfg_num_tiles  - 8'd1);
    assign yout_last  = (yout_cnt == cfg_h_out      - 16'd1);
    assign cs_last    = (cs_cnt   == cfg_cout_slices- 6'd1);
    assign round_last = (round_cnt == cfg_rounds_per_cins - 3'd1);
    assign pos_last   = (pos_in_round == cur_round_len - 6'd1);

    // cins 边界 = 轮内最后一拍 && 轮数到末尾 (也等价 ky_last && kx_last)
    logic cins_end;
    assign cins_end = subop_last_cycle && ky_last && kx_last;

    // 轮次边界（非 cins 边界）= 轮最后一拍 but still 有下一轮
    logic round_end_only;
    assign round_end_only = subop_last_cycle && pos_last && !round_last;

    // tile 内最后一个 sub_op
    logic last_subop_in_tile;
    assign last_subop_in_tile = cins_last && ky_last && kx_last;

    //=========================================================================
    // 9. FSM 时序
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            cs_cnt     <= '0;
            yout_cnt   <= '0;
            tile_cnt   <= '0;
            cins_cnt   <= '0;
            ky_cnt     <= '0;
            kx_cnt     <= '0;
            round_cnt  <= '0;
            pos_in_round <= '0;
            sub_cnt    <= '0;
            first_tile_subop <= 1'b1;

            r_ifb_yout <= '0;
            r_ifb_cins <= '0;
            r_ifb_ky   <= '0;
            r_ofb_cs   <= '0;
            r_ofb_yout <= '0;
            r_wb_cs    <= '0;
            r_wb_cins  <= '0;
            x_tile_out_r <= '0;
            x_tile_in_r  <= '0;
            r_wrf_base <= '0;

            wrf_we_d1    <= '0; wrf_waddr_d1 <= '0;
            arf_we_d1    <= 1'b0; arf_waddr_d1 <= '0;
            ofb_we_d1    <= 1'b0; ofb_waddr_d1 <= '0; sdp_en_d1 <= 1'b0;
            ofb_we_d2    <= 1'b0; ofb_waddr_d2 <= '0; sdp_en_d2 <= 1'b0;
        end else begin
            state <= nstate;

            //---- 默认脉冲信号清零 ----
            wrf_we_d1    <= '0;
            arf_we_d1    <= 1'b0;
            ofb_we_d1    <= 1'b0;
            sdp_en_d1    <= 1'b0;

            //---- IDLE → start：初始化 ----
            if (state == S_IDLE && start) begin
                cs_cnt   <= '0;
                yout_cnt <= '0;
                tile_cnt <= '0;
                cins_cnt <= '0;
                ky_cnt   <= '0;
                kx_cnt   <= '0;
                round_cnt <= '0;
                pos_in_round <= '0;
                sub_cnt  <= '0;
                first_tile_subop <= 1'b1;

                r_ifb_yout <= cfg_ifb_base;
                r_ifb_cins <= cfg_ifb_base;
                r_ifb_ky   <= cfg_ifb_base;
                r_ofb_cs   <= cfg_ofb_base;
                r_ofb_yout <= cfg_ofb_base;
                r_wb_cs    <= cfg_wb_base;
                r_wb_cins  <= cfg_wb_base;
                x_tile_out_r <= '0;
                x_tile_in_r  <= '0;
                r_wrf_base <= '0;
            end

            //---- S_LOAD_WGT ----
            if (state == S_LOAD_WGT) begin
                // 延迟 1 拍写 WRF
                wrf_we_d1    <= '1;
                wrf_waddr_d1 <= sub_cnt[WAW-1:0];

                if (sub_cnt == (cfg_wrf_packed ? cfg_total_wrf - 10'd1
                                                : {4'd0, cur_round_len} - 10'd1)) begin
                    sub_cnt <= '0;
                end else begin
                    sub_cnt <= sub_cnt + 16'd1;
                end
            end

            //---- S_SUBOP ----
            if (state == S_SUBOP) begin
                // ARF 写（d1）
                if (sub_cnt < {10'd0, sub_ld_len}) begin
                    arf_we_d1    <= 1'b1;
                    arf_waddr_d1 <= sub_arf_wr_base + sub_cnt[AAW-1:0];
                end

                if (sub_cnt == sub_end_cnt) begin
                    // sub_op 结束，清零内部 cnt
                    sub_cnt <= '0;
                    first_tile_subop <= 1'b0;

                    //---- 推进 ky/kx ----
                    if (!(kx_last && ky_last)) begin
                        // 非 cins 边界：ky/kx 正常推进
                        if (!kx_last) begin
                            kx_cnt <= kx_cnt + 4'd1;
                        end else begin
                            kx_cnt <= 4'd0;
                            ky_cnt <= ky_cnt + 4'd1;
                            r_ifb_ky <= r_ifb_ky + {{(ADDR_W-16){1'b0}}, cfg_w_in};
                        end
                    end else begin
                        // cins 边界 (kx_last && ky_last)
                        kx_cnt <= 4'd0;
                        ky_cnt <= 4'd0;
                    end

                    //---- 推进 pos_in_round / round_cnt ----
                    if (pos_last) begin
                        pos_in_round <= 6'd0;
                        if (round_last) begin
                            // 轮 & cins 均到末 → cins 边界
                            round_cnt <= 3'd0;
                            if (!cins_last) begin
                                cins_cnt <= cins_cnt + 6'd1;
                                // packed: r_wrf_base 加 kk；chunked: 保持 0
                                if (cfg_wrf_packed) begin
                                    r_wrf_base <= r_wrf_base + cfg_kk[WAW-1:0];
                                end
                                r_ifb_cins <= r_ifb_cins + cfg_ifb_cin_step;
                                r_ifb_ky   <= r_ifb_cins + cfg_ifb_cin_step;
                                r_wb_cins  <= r_wb_cins + cfg_wb_cin_step;
                            end else begin
                                // tile 边界：让 ST_OFM 处理推进
                                cins_cnt  <= 6'd0;
                                r_wrf_base <= '0;
                                r_ifb_cins <= r_ifb_yout;
                                r_ifb_ky   <= r_ifb_yout;
                                r_wb_cins  <= r_wb_cs;
                            end
                        end else begin
                            // 轮内边界，cins 不变
                            round_cnt <= round_cnt + 3'd1;
                        end
                    end else begin
                        pos_in_round <= pos_in_round + 6'd1;
                    end
                end else begin
                    sub_cnt <= sub_cnt + 16'd1;
                end
            end

            //---- S_ST_OFM ----
            if (state == S_ST_OFM) begin
                if (sub_cnt < valid_w) begin
                    logic [ADDR_W-1:0] ofb_addr_full;
                    ofb_addr_full = r_ofb_yout + x_tile_out_r + {{(ADDR_W-16){1'b0}}, sub_cnt};
                    ofb_we_d1    <= 1'b1;
                    ofb_waddr_d1 <= ofb_addr_full[AW-1:0];
                    sdp_en_d1    <= cfg_sdp_relu_en;
                end

                if (sub_cnt == valid_w - 16'd1) begin
                    sub_cnt <= '0;
                    first_tile_subop <= 1'b1;

                    if (!tile_last) begin
                        tile_cnt     <= tile_cnt + 8'd1;
                        x_tile_out_r <= x_tile_out_r + {{(ADDR_W-6){1'b0}}, cfg_tile_w};
                        x_tile_in_r  <= x_tile_in_r  + cfg_tile_in_step;
                    end else begin
                        tile_cnt     <= 8'd0;
                        x_tile_out_r <= '0;
                        x_tile_in_r  <= '0;
                        if (!yout_last) begin
                            yout_cnt   <= yout_cnt + 16'd1;
                            r_ifb_yout <= r_ifb_yout + cfg_ifb_row_step;
                            r_ifb_cins <= r_ifb_yout + cfg_ifb_row_step;
                            r_ifb_ky   <= r_ifb_yout + cfg_ifb_row_step;
                            r_ofb_yout <= r_ofb_yout + {{(ADDR_W-16){1'b0}}, cfg_w_out};
                        end else begin
                            yout_cnt   <= 16'd0;
                            if (!cs_last) begin
                                cs_cnt     <= cs_cnt + 6'd1;
                                r_ifb_yout <= cfg_ifb_base;
                                r_ifb_cins <= cfg_ifb_base;
                                r_ifb_ky   <= cfg_ifb_base;
                                r_wb_cs    <= r_wb_cs + cfg_wb_cout_step;
                                r_wb_cins  <= r_wb_cs + cfg_wb_cout_step;
                                r_ofb_cs   <= r_ofb_cs + cfg_ofb_cout_step;
                                r_ofb_yout <= r_ofb_cs + cfg_ofb_cout_step;
                                r_wrf_base <= '0;
                            end
                        end
                    end
                end else begin
                    sub_cnt <= sub_cnt + 16'd1;
                end
            end

            if (state == S_SHIFT_WR) begin
                sub_cnt <= '0;
            end

            //---- d2 延迟 ----
            ofb_we_d2    <= ofb_we_d1;
            ofb_waddr_d2 <= ofb_waddr_d1;
            sdp_en_d2    <= sdp_en_d1;
        end
    end

    //=========================================================================
    // 10. nstate
    //=========================================================================
    always_comb begin
        nstate = state;
        case (state)
            S_IDLE:     if (start) nstate = S_SHIFT_WR;
            S_SHIFT_WR:            nstate = S_LOAD_WGT;
            S_LOAD_WGT: if (sub_cnt == (cfg_wrf_packed ? cfg_total_wrf - 10'd1
                                                       : {4'd0, cur_round_len} - 10'd1))
                                               nstate = S_SUBOP;
            S_SUBOP:    begin
                if (subop_last_cycle) begin
                    if (last_subop_in_tile)
                        nstate = S_ST_OFM;
                    else if (cins_end && !cins_last && !cfg_wrf_packed)
                        nstate = S_LOAD_WGT;   // chunked: 下一 cins，round 0
                    else if (round_end_only)
                        nstate = S_LOAD_WGT;   // chunked: 同 cins 下一 round
                    else
                        nstate = S_SUBOP;      // 继续下一 sub_op (同 round)
                end
            end
            S_ST_OFM:   if (sub_cnt == valid_w - 16'd1) begin
                            if (tile_last && yout_last && cs_last)
                                nstate = S_DONE;
                            else if (cfg_wrf_packed) begin
                                if (tile_last && yout_last)
                                    nstate = S_LOAD_WGT;  // 下一 cs 重装
                                else
                                    nstate = S_SUBOP;
                            end else begin
                                nstate = S_LOAD_WGT;  // chunked：每新 tile 都要重装
                            end
                        end
            S_DONE:                             nstate = S_DONE;
            default:                            nstate = S_IDLE;
        endcase
    end

    //=========================================================================
    // 11. 组合输出
    //=========================================================================
    assign done = (state == S_DONE);

    // WB 地址（LOAD_WGT 期间）
    logic [ADDR_W-1:0] wb_raddr_full;
    always_comb begin
        if (cfg_wrf_packed)
            wb_raddr_full = r_wb_cs + {{(ADDR_W-16){1'b0}}, sub_cnt};
        else
            // chunked：r_wb_cins + round_cnt*32 + sub_cnt
            wb_raddr_full = r_wb_cins
                          + {{(ADDR_W-8){1'b0}}, round_cnt, 5'd0}
                          + {{(ADDR_W-16){1'b0}}, sub_cnt};
    end
    always_comb begin
        wb_re    = 1'b0;
        wb_raddr = '0;
        if (state == S_LOAD_WGT) begin
            wb_re    = 1'b1;
            wb_raddr = wb_raddr_full[AW-1:0];
        end
    end
    assign wrf_we    = wrf_we_d1;
    assign wrf_waddr = wrf_waddr_d1;

    // IFB 地址（SUB_OP 期间 cnt<ld_len）
    logic [15:0] ifb_step_off;
    assign ifb_step_off = {13'd0, sub_eff_stride} * sub_cnt;

    logic [ADDR_W-1:0] ifb_raddr_full;
    assign ifb_raddr_full = r_ifb_ky + x_tile_in_r
                          + {{(ADDR_W-16){1'b0}}, sub_ifb_offset}
                          + {{(ADDR_W-16){1'b0}}, ifb_step_off};
    always_comb begin
        ifb_re    = 1'b0;
        ifb_raddr = '0;
        if (state == S_SUBOP && sub_cnt < {10'd0, sub_ld_len}) begin
            ifb_re    = 1'b1;
            ifb_raddr = ifb_raddr_full[AW-1:0];
        end
    end

    assign arf_we    = arf_we_d1;
    assign arf_waddr = arf_waddr_d1;

    // MAC / PARF（SUB_OP cnt=1..mac_len）
    always_comb begin
        compute_en    = 1'b0;
        arf_read_addr = '0;
        wrf_raddr     = '0;
        parf_addr     = '0;
        parf_we       = 1'b0;
        parf_clear    = 1'b0;

        if (state == S_SUBOP && sub_cnt > 16'd0) begin
            compute_en    = 1'b1;
            arf_read_addr = sub_arf_rd_base + sub_cnt[AAW-1:0] - {{(AAW-1){1'b0}},1'b1};
            // wrf_raddr = r_wrf_base + pos_in_round (均 ≤ 31)
            wrf_raddr     = r_wrf_base + pos_in_round[WAW-1:0];
            parf_addr     = sub_cnt[PAW-1:0] - {{(PAW-1){1'b0}},1'b1};
            parf_we       = 1'b1;
            parf_clear    = first_tile_subop;
        end else if (state == S_ST_OFM) begin
            parf_addr     = sub_cnt[PAW-1:0];
        end
    end

    assign ofb_we     = ofb_we_d2;
    assign ofb_waddr  = ofb_waddr_d2;
    assign sdp_en_out = sdp_en_d2;

    assign sdp_shift_we    = (state == S_SHIFT_WR);
    assign sdp_shift_wdata = cfg_sdp_shift;

endmodule
