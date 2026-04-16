// -----------------------------------------------------------------------------
// File        : sdp.sv
// Description : Single Data Processor (SDP)
//               Post-processing pipeline after MAC accumulation:
//               1. Dequantize : arithmetic right-shift by shift_amt (0..31)
//               2. Activation : optional ReLU (max(0, x))
//               3. Requantize : clip to [0, 255] -> uint8 output
//
//               valid_in / valid_out are pipeline handshake signals.
//               In the current sequential-MAC design they are purely
//               combinational pass-throughs (zero latency). The signals are
//               exposed so a future async SDP engine can add FIFO / buffering
//               without changing the interface.
//
// Parameters  : NUM_COL    - number of output channels (= MAC array columns)
//               PSUM_WIDTH - width of each partial-sum input (signed, int32)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps

module sdp #(
    parameter int NUM_COL    = 8,
    parameter int PSUM_WIDTH = 32
)(
    // --- Parameter configuration (静态，由 cfg_regs 直接驱动) ---
    input  logic [4:0]                        shift_amt,      // arithmetic right-shift amount

    // --- Data path ---
    // Input: packed partial sums from PARF, one per column
    input  logic [NUM_COL*PSUM_WIDTH-1:0]     psum_in,
    input  logic                              valid_in,       // data on psum_in is valid
    input  logic                              relu_en,        // 1 = apply ReLU, 0 = bypass

    // Output: packed uint8 pixels, one per column
    output logic [NUM_COL*8-1:0]              ofm_data,
    output logic                              valid_out
);

    // -------------------------------------------------------------------------
    // Per-channel intermediate signals (declared outside always_comb)
    // -------------------------------------------------------------------------
    logic signed [PSUM_WIDTH-1:0] psum_ch  [0:NUM_COL-1]; // extracted signed psum
    logic signed [PSUM_WIDTH-1:0] dq_ch    [0:NUM_COL-1]; // after dequant shift
    logic signed [PSUM_WIDTH-1:0] act_ch   [0:NUM_COL-1]; // after ReLU

    // -------------------------------------------------------------------------
    // Per-channel processing (purely combinational, zero added latency)
    // -------------------------------------------------------------------------
    always_comb begin
        valid_out = valid_in;
        for (int c = 0; c < NUM_COL; c++) begin
            // 1. Extract signed partial sum for this channel
            psum_ch[c] = $signed(psum_in[c*PSUM_WIDTH +: PSUM_WIDTH]);

            // 2. Dequantize: arithmetic right-shift (signed >> preserves sign bit)
            dq_ch[c] = psum_ch[c] >>> shift_amt;

            // 3. Activation: ReLU (clamp negatives to 0)
            act_ch[c] = (relu_en && dq_ch[c] < 0) ? '0 : dq_ch[c];

            // 4. Requantize: clip to [0, 255] -> 8-bit unsigned
            if (act_ch[c] < 0)
                ofm_data[c*8 +: 8] = 8'd0;
            else if (act_ch[c] > 255)
                ofm_data[c*8 +: 8] = 8'd255;
            else
                ofm_data[c*8 +: 8] = act_ch[c][7:0];
        end
    end

endmodule
