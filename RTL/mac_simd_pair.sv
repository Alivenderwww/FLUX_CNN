`timescale 1ns/1ps

// =============================================================================
// mac_simd_pair.sv  —  INT8 SIMD on DSP48E1 (1 DSP 装 2 个 PE)
//
// 把 2 个共享 input 但 weight 不同的 PE 合到 1 个 DSP48E1, 利用 25×18 multiplier
// 算 2 个独立 8×8 mul. 适用于"输入广播+权重独立"的位置 (跨 mac_col 同 row 配对).
//
// Packing scheme (XAPP1259 风格):
//   A_packed (25-bit) = {w1[7:0], 9'b0, w0[7:0]}   <- 两 weight + 9-bit carry isolation
//   B_ext    (18-bit) = sign_extend(act_in[7:0], 18)
//   P (43-bit)        = A_packed × B_ext
//
//   P[15:0]  contains  (w0_unsigned × B)           — 需 sign correction
//   P[33:18] contains  (w1 × B + carry from w0部分)  — carry isolation 9-bit 吸收 borrow
//
// Sign correction (因 packing 把 w0 当 unsigned 9-bit 0..255):
//   当 w0 < 0 时, A_low_8 = w0 + 256 (unsigned 视图), 多算了 256 × B
//   所以 prod_0 = P[15:0] - (w0[7] ? (act_in << 8) : 0)
//   prod_1 (高位) carry isolation 9-bit 已吸收, 直接 P[33:18] = w1 × B (无需 correction)
//
// 注: 当前先用 RTL 推断 (use_dsp = "yes"), Vivado 看能否推 DSP48E1. 若推不出再
//     改成手动实例化 DSP48E1 原语 (UG479 Table 2-2 ports).
//
// 时序: compute_en T → dsp_p reg T+1 → sign_corr LUT reg T+2 → prod_0/prod_1 输出 T+2
//   比原 mac_pe (T+1 prod_out) 多 1 拍, 因 sign correction 必须独立 reg 阻断 DSP cascade.
//   (sign corr 直接组合到 dsp_p 上时, Vivado 会把 reg+sub 推断成 DSP C-A:B 模式, 占额外 DSP)
//   mac_array_simd pipe 跟着改 3-stage (s1=dsp_p, s2=sign_corr, s3=adder_tree_reg).
// =============================================================================
module mac_simd_pair #(
    parameter int DATA_WIDTH = 8,
    parameter int WRF_DEPTH  = 32,
    parameter int PROD_WIDTH = 17    // 16-bit mul + 1-bit sign correction headroom
)(
    input  logic                               clk,
    input  logic                               rst_n,

    // 2 个独立 WRF (PE 0 / PE 1)
    input  logic [1:0]                         wrf_we,    // [0]=PE0, [1]=PE1
    input  logic [$clog2(WRF_DEPTH)-1:0]       wrf_waddr,
    input  logic [DATA_WIDTH-1:0]              wrf_wdata_0,
    input  logic [DATA_WIDTH-1:0]              wrf_wdata_1,

    input  logic [$clog2(WRF_DEPTH)-1:0]       wrf_raddr,
    input  logic signed [DATA_WIDTH-1:0]       act_in,
    input  logic                               compute_en,

    // 2 个独立 prod 输出 (signed)
    output logic signed [PROD_WIDTH-1:0]       prod_0,
    output logic signed [PROD_WIDTH-1:0]       prod_1
);

    // -------------------------------------------------------------------------
    // 1. 2 个 WRF (复用 std_rf, 跟原 mac_pe 一致)
    // -------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] w0_us, w1_us;

    std_rf #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(WRF_DEPTH)
    ) u_wrf_0 (
        .clk   (clk),    .rst_n(rst_n),
        .we    (wrf_we[0]),
        .waddr (wrf_waddr), .wdata(wrf_wdata_0),
        .raddr (wrf_raddr), .rdata(w0_us)
    );

    std_rf #(
        .DATA_WIDTH(DATA_WIDTH),
        .DEPTH(WRF_DEPTH)
    ) u_wrf_1 (
        .clk   (clk),    .rst_n(rst_n),
        .we    (wrf_we[1]),
        .waddr (wrf_waddr), .wdata(wrf_wdata_1),
        .raddr (wrf_raddr), .rdata(w1_us)
    );

    logic signed [DATA_WIDTH-1:0] w0, w1;
    assign w0 = signed'(w0_us);
    assign w1 = signed'(w1_us);

    // -------------------------------------------------------------------------
    // 2. Pack 2 weights into 25-bit A, sign-extend act_in to 18-bit B
    //    A = {w1[7:0], 9'b0, w0[7:0]}  (signed view, bit 24 = w1[7] = sign of w1)
    //    B = sign_extend(act_in, 18)
    // -------------------------------------------------------------------------
    logic signed [24:0] dsp_a;
    logic signed [17:0] dsp_b;
    assign dsp_a = $signed({w1, 9'b0, w0});
    assign dsp_b = $signed({{10{act_in[7]}}, act_in});

    // -------------------------------------------------------------------------
    // 3. Multiplier (use_dsp attribute 让 Vivado 推 DSP48E1)
    //    P = A × B, registered (1 拍 mul + 1 拍 P reg)
    // -------------------------------------------------------------------------
    (* use_dsp = "yes" *) logic signed [42:0] dsp_p;
`ifdef FLUX_MAC_BYPASS
    // VD100 isolate sim: bypass DSP 乘法, 省 384 DSP + 加速 P&R. dsp_p = dsp_b (passthrough)
    always_ff @(posedge clk) begin
        if (compute_en) dsp_p <= {{25{dsp_b[17]}}, dsp_b};   // 18-bit signed → 43-bit
    end
    wire _unused_a = |dsp_a;
`else
    always_ff @(posedge clk) begin
        if (compute_en) dsp_p <= dsp_a * dsp_b;
    end
`endif

    // -------------------------------------------------------------------------
    // 4. Sign correction (组合逻辑输出, 不加 reg)
    //    需要 pipeline w0_sign / w0_nonzero / act_in 一拍 (跟 dsp_p 同一时刻有效)
    // -------------------------------------------------------------------------
    logic                          w0_sign_pipe;
    logic                          w0_nonzero_pipe;
    logic signed [DATA_WIDTH-1:0]  act_in_pipe;
    always_ff @(posedge clk) begin
        if (compute_en) begin
            w0_sign_pipe    <= w0[DATA_WIDTH-1];
            w0_nonzero_pipe <= |w0;
            act_in_pipe     <= act_in;
        end
    end

    // -------------------------------------------------------------------------
    // 5. 提取 prod_0 / prod_1 + sign correction (组合)
    //
    //    Packing: A = w1<<17 + w0_unsigned (where w0_unsigned = w0 if w0≥0 else w0+256)
    //    A × B   = w1×B<<17 + w0_unsigned×B
    //    w0_unsigned×B 范围 [-32640, 32385], 全在 17-bit 内, 不直接污染高位.
    //    但当 w0_unsigned×B < 0 (w0≠0 且 B<0) 时, 它的 sign extension 会让
    //    arithmetic shift right by 17 给 -1, 即 P[32:17] = w1×B - 1, 需 +1 修正.
    //
    //    prod_0 = P[15:0]_signed - (w0_sign ? 256×B : 0)
    //    prod_1 = P[32:17]_signed + (w0_nonzero & act<0 ? 1 : 0)
    // -------------------------------------------------------------------------
    logic signed [16:0] prod_0_raw;       // P[15:0] sign-ext to 17-bit
    logic signed [16:0] prod_0_corr;      // (act_in << 8) sign-ext to 17-bit (= 256 × act_in)
    logic signed [15:0] prod_1_raw;       // P[32:17]
    logic               prod_1_corr_bit;  // w0_nonzero & act_sign

    assign prod_0_raw  = $signed({dsp_p[15], dsp_p[15:0]});
    assign prod_0_corr = w0_sign_pipe ? $signed({act_in_pipe, 8'b0}) : '0;
    assign prod_1_raw  = $signed(dsp_p[32:17]);
    assign prod_1_corr_bit = w0_nonzero_pipe & act_in_pipe[DATA_WIDTH-1];

    // sign correction 走独立 module + module-level USE_DSP="no" 强制 LUT 实现.
    // (USE_DSP 加在 logic decl/always_ff/wire 上 Vivado 全不识别, 必须 module-level)
    sign_corr_lut u_corr (
        .clk          (clk),
        .compute_en   (compute_en),
        .prod_0_raw   (prod_0_raw),
        .prod_0_corr  (prod_0_corr),
        .prod_1_raw_se({prod_1_raw[15], prod_1_raw}),
        .prod_1_corr_bit(prod_1_corr_bit),
        .prod_0       (prod_0),
        .prod_1       (prod_1)
    );

endmodule


// =============================================================================
// sign_corr_lut  --  独立 module 强制 sign correction 走 LUT (避免被 Vivado 推 DSP)
//   module-level (* USE_DSP = "no" *) 是唯一对 Vivado 真正生效的写法.
//   加在 always_ff / logic decl / wire 上都被 Vivado 忽略.
// =============================================================================
(* USE_DSP = "no" *)
module sign_corr_lut (
    input  logic               clk,
    input  logic               compute_en,
    input  logic signed [16:0] prod_0_raw,
    input  logic signed [16:0] prod_0_corr,
    input  logic signed [16:0] prod_1_raw_se,
    input  logic               prod_1_corr_bit,
    output logic signed [16:0] prod_0,
    output logic signed [16:0] prod_1
);
    always_ff @(posedge clk) begin
        if (compute_en) begin
            prod_0 <= prod_0_raw - prod_0_corr;
            prod_1 <= prod_1_raw_se + $signed({16'd0, prod_1_corr_bit});
        end
    end
endmodule
