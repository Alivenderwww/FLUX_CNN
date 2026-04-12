// -----------------------------------------------------------------------------
// File        : core_isa_pkg.sv
// Description : Instruction Set Architecture definition for the CNN Macro-Core
// -----------------------------------------------------------------------------
package core_isa_pkg;

    // --- 1. Opcode Definition (4-bit) ---
    typedef enum logic [3:0] {
        OP_NOP       = 4'd0,  // No Operation / Wait
        OP_LD_WGT    = 4'd1,  // Load Weights into PE Array Weight RF
        OP_LD_ARF    = 4'd2,  // Load Data into ARF Shift Register from Ifmap Buffer
        OP_LD_PARF   = 4'd3,  // Load Initial Psum into PARF
        OP_ST_OFM    = 4'd4,  // Store Output Feature Map from PARF to Outmap Buffer
        OP_MAC_RUN   = 4'd5   // Execute MAC Pipeline
    } opcode_e;

    // --- 2. Instruction Format (64-bit) ---
    typedef struct packed {
        opcode_e       opcode;      // [63:60] Opcode
        
        // --- Control Flags ---
        logic          shift_arf;   // [59]    OP_MAC_RUN: Shift ARF before calculation (kx=1,2)
        logic          clr_parf;    // [58]    OP_MAC_RUN: Clear PARF before accumulate (kx=0, ky=0, cin=0)
        logic          sdp_en;      // [57]    OP_ST_OFM: Enable Single Data Processor (ReLU/Dequantize)
        
        // --- Internal Register Addressing ---
        logic [4:0]    wgt_rf_addr; // [56:52] (5-bit) Address in PE's 32-depth Weight RF (0~31)
        logic [4:0]    parf_addr;   // [51:47] (5-bit) Address in PARF (0~31)
        
        // --- External/Buffer Addressing & Control ---
        logic [14:0]   sram_addr;   // [46:32] (15-bit) Base address in Ifmap/Outmap Buffer
        logic [15:0]   length;      // [31:16] (16-bit) Length of continuous operation (e.g., 32 cycles)
        logic [15:0]   stride;      // [15:0]  (16-bit) Stride for 2D wrapping or reserved
    } inst_t;

endpackage : core_isa_pkg
