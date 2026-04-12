`timescale 1ns/1ps

module tb_core_isa;

    // Parameters
    localparam NUM_COL    = 8;
    localparam PSUM_WIDTH = 32;
    localparam SRAM_DEPTH = 1024;
    localparam INST_DEPTH = 1024;

    logic clk;
    logic rst_n;
    logic start;
    logic done;
    logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec;

    // Instantiate core
    core_top #(
        .NUM_COL(NUM_COL),
        .NUM_PE(8),
        .DATA_WIDTH(8),
        .PSUM_WIDTH(PSUM_WIDTH),
        .WRF_DEPTH(32),
        .ARF_DEPTH(32),
        .PARF_DEPTH(32),
        .SRAM_DEPTH(SRAM_DEPTH),
        .INST_DEPTH(INST_DEPTH)
    ) u_core_top (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .inst_we_ext(1'b0),
        .inst_waddr_ext('0),
        .inst_wdata_ext('0),
        .ifb_we_ext(1'b0),
        .ifb_waddr_ext('0),
        .ifb_wdata_ext('0),
        .wb_we_ext(1'b0),
        .wb_waddr_ext('0),
        .wb_wdata_ext('0),
        .psum_out_vec(psum_out_vec),
        .done(done)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Load SRAMs
    initial begin
        $readmemh("inst.txt", u_core_top.u_inst_sram.mem);
        $readmemh("ifb.txt",  u_core_top.u_ifb.mem);
        $readmemh("wb.txt",   u_core_top.u_wb.mem);
    end

    // Output probing array
    logic signed [31:0] parf_vals [32][8];
    genvar c_idx, p_idx;
    generate
        for (c_idx = 0; c_idx < 8; c_idx++) begin : gen_probe_c
            for (p_idx = 0; p_idx < 32; p_idx++) begin : gen_probe_p
                assign parf_vals[p_idx][c_idx] = u_core_top.u_mac_array.gen_col[c_idx].u_col.u_parf.ram[p_idx];
            end
        end
    endgenerate

    // Expected Psum array
    logic signed [31:0] expected_psum_arr [0:255]; // 32 pixels * 8 channels

    // Test sequence
    initial begin
        rst_n = 0;
        start = 0;
        #20;
        rst_n = 1;
        #10;
        
        // Load expected values
        $readmemh("expected_psum.txt", expected_psum_arr);
        
        $display("========================================");
        $display("Starting Macro-ISA CNN Core...");
        $display("========================================");
        
        start = 1;
        #10;
        start = 0;

        // Wait for done flag
        wait(done == 1);
        #50;

        $display("========================================");
        $display("Macro-ISA Core Execution Finished.");
        $display("Checking PARF internally computed results...");
        
        begin
            // Count total mismatches
            automatic int mismatch_cnt = 0;
            
            // Verify results directly from the PARF registers inside MAC Array
            for (int p = 0; p < 32; p++) begin
                for (int c = 0; c < 8; c++) begin
                    // Hierarchical reference to PARF via continuous assignment array
                    automatic logic signed [31:0] actual = parf_vals[p][c];
                    
                    // Read expected Psum
                    automatic logic signed [31:0] expected = expected_psum_arr[p * 8 + c];
                    
                    if (actual !== expected) begin
                        if (1) begin
                            $display("FAIL: Pixel[%0d], Output Channel[%0d] | Expected: %0d, Got: %0d", p, c, expected, actual);
                        end
                        if (mismatch_cnt == 2) begin
                            $display("... (Further mismatches suppressed)");
                        end
                        mismatch_cnt++;
                    end
                end
            end
            
            if (mismatch_cnt == 0) begin
                $display("Check Complete! 0 mismatches found. ISA simulation passed successfully.");
            end else begin
                $display("Check Complete! Found %0d mismatches. Simulation FAILED.", mismatch_cnt);
            end
        end
        $display("========================================");
        $stop;
    end

endmodule