// -----------------------------------------------------------------------------
// File        : core_isa_pkg.sv
// Description : Instruction Set Architecture definition for the CNN Macro-Core
// -----------------------------------------------------------------------------
package core_isa_pkg;

    // --- 1. Opcode Definition (4-bit) ---
    typedef enum logic [3:0] {
        OP_NOP       = 4'd0,  // No Operation / Wait
        OP_LD_WGT    = 4'd1,  // Load Weights into PE Array Weight RF
        OP_LD_ARF    = 4'd2,  // Load Data into ARF from Ifmap Buffer
        OP_LD_PARF   = 4'd3,  // Load Initial Psum into PARF
        OP_ST_OFM    = 4'd4,  // Store Output Feature Map from PARF to Outmap Buffer
        OP_MAC_RUN   = 4'd5,  // Execute MAC Pipeline
        OP_LD1MAC    = 4'd6,  // Load 1 pixel to ARF[ld_arf_addr] AND run MAC simultaneously (kx-switch optimization)
        OP_LD32MAC   = 4'd7   // Load N pixels from IFB into ARF AND run MAC simultaneously (initial-load optimization)
                              // State runs length+1 cycles: cycle 0 = IFB prefetch, cycles 1..length = load+MAC overlap
    } opcode_e;

    // --- 2. Instruction Format (64-bit) ---
    // Bit layout (MSB→LSB):
    //   [63:60] opcode      [59] clr_parf   [58] sdp_en
    //   [57:53] arf_addr    [52:48] wgt_rf_addr  [47:43] parf_addr
    //   [42:28] sram_addr   [27:12] length   [11:5] stride_hi  [4:0] ld_arf_addr
    //
    // ld_arf_addr (stride[4:0]): OP_LD1MAC 专用 —— 单像素写入 ARF 的目标地址
    typedef struct packed {
        opcode_e       opcode;      // [63:60] Opcode

        // --- Control Flags ---
        logic          clr_parf;    // [59]    OP_MAC_RUN/OP_LD1MAC: Clear PARF before accumulate
        logic          sdp_en;      // [58]    OP_ST_OFM: Enable Single Data Processor (ReLU/Dequantize)

        // --- Internal Register Addressing ---
        logic [4:0]    arf_addr;    // [57:53] (5-bit) MAC read base address in ARF (0~31)
        logic [4:0]    wgt_rf_addr; // [52:48] (5-bit) Address in PE's 32-depth Weight RF (0~31)
        logic [4:0]    parf_addr;   // [47:43] (5-bit) Address in PARF (0~31)

        // --- External/Buffer Addressing & Control ---
        logic [14:0]   sram_addr;   // [42:28] (15-bit) Base address in Ifmap/Outmap Buffer
        logic [15:0]   length;      // [27:12] (16-bit) Length of continuous operation (e.g., 32 cycles)
        logic [6:0]    stride_hi;   // [11:5]  Reserved
        logic [4:0]    ld_arf_addr; // [4:0]   OP_LD1MAC: ARF write address for the single loaded pixel
    } inst_t;

endpackage : core_isa_pkg
