`timescale 1ns/1ps

// 核心顶层测试平台 (Testbench)
// 用于验证数据流与Psum累加的正确性
module tb_core_top;

    parameter int NUM_COL     = 8;
    parameter int NUM_PE      = 8;
    parameter int DATA_WIDTH  = 8;
    parameter int PSUM_WIDTH  = 32;
    parameter int SRAM_DEPTH  = 1024;
    
    logic                                clk;
    logic                                rst_n;
    
    logic                                start;
    logic [3:0]                          kernel_size;
    logic [3:0]                          kernel_dim;
    logic [15:0]                         image_width;
    logic [7:0]                          num_pixels;
    
    // SRAM加载信号
    logic                                ifb_we_ext;
    logic [$clog2(SRAM_DEPTH)-1:0]       ifb_waddr_ext;
    logic [63:0]                         ifb_wdata_ext;
    
    logic                                wb_we_ext;
    logic [$clog2(SRAM_DEPTH)-1:0]       wb_waddr_ext;
    logic [511:0]                        wb_wdata_ext;
    
    logic signed [NUM_COL*PSUM_WIDTH-1:0] psum_out_vec;
    logic                                 done;
    
    // 实例化顶层
    core_top #(
        .NUM_COL(NUM_COL),
        .NUM_PE(NUM_PE),
        .DATA_WIDTH(DATA_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .WRF_DEPTH(32),
        .ARF_DEPTH(32),
        .PARF_DEPTH(32),
        .SRAM_DEPTH(SRAM_DEPTH)
    ) dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .kernel_size    (kernel_size),
        .kernel_dim     (kernel_dim),
        .image_width    (image_width),
        .num_pixels     (num_pixels),
        .ifb_we_ext     (ifb_we_ext),
        .ifb_waddr_ext  (ifb_waddr_ext),
        .ifb_wdata_ext  (ifb_wdata_ext),
        .wb_we_ext      (wb_we_ext),
        .wb_waddr_ext   (wb_waddr_ext),
        .wb_wdata_ext   (wb_wdata_ext),
        .psum_out_vec   (psum_out_vec),
        .done           (done)
    );

    //=============================================================================
    // 1. 时钟和复位
    //=============================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns 周期 (100MHz)，逻辑验证使用
    end
    
    initial begin
        rst_n = 0;
        #20 rst_n = 1;
    end

    // 动态文件读取数据
    logic [63:0] ifm_mem [0:1023];
    logic [511:0] weight_mem [0:8]; // Up to 3x3=9 weights
    logic [255:0] ofm_expected [0:1023];
    
    // Configurable parameters derived from run
    int H_IN = 3;
    int W_IN = 34;
    int K = 3;
    int H_OUT, W_OUT, NUM_PIXELS_TO_COMPUTE;
    
    // ------------------------------------------------------------------------
    // Performance Counters
    // ------------------------------------------------------------------------
    int cnt_ifb_wr = 0;
    int cnt_ifb_rd = 0;
    int cnt_wb_wr  = 0;
    int cnt_wb_rd  = 0;
    int cnt_wrf_wr = 0;
    int cnt_wrf_rd = 0;
    int cnt_arf_wr = 0;
    int cnt_arf_rd = 0;
    int cnt_parf_wr = 0;
    int cnt_parf_rd = 0;

    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (ifb_we_ext) cnt_ifb_wr++;
            if (dut.ifb_re) cnt_ifb_rd++;
            
            if (wb_we_ext) cnt_wb_wr++;
            if (dut.wb_re) cnt_wb_rd++;
            
            if (dut.wrf_we != 0) cnt_wrf_wr++;
            if (dut.compute_en) cnt_wrf_rd++;
            
            if (dut.arf_flush_en || dut.arf_shift_en) cnt_arf_wr++;
            if (dut.compute_en) cnt_arf_rd++;
            
            if (dut.parf_we) cnt_parf_wr++;
            if (dut.compute_en) cnt_parf_rd++;
        end
    end
    
    initial begin
        // 读取文件
        $readmemh("ifm.txt", ifm_mem);
        $readmemh("weight.txt", weight_mem);
        $readmemh("ofm.txt", ofm_expected);
        
        H_OUT = H_IN - K + 1;
        W_OUT = W_IN - K + 1;
        NUM_PIXELS_TO_COMPUTE = H_OUT * W_OUT;
    end
    
    //=============================================================================
    // 2. 激励加载和流程控制
    //=============================================================================
    initial begin
        // 初始化外部信号
        start         = 0;
        kernel_size   = K * K;  // 例如 3x3=9, 1x1=1
        kernel_dim    = K;      // 边长
        image_width   = W_IN;   // 输入特征图宽度
        num_pixels    = NUM_PIXELS_TO_COMPUTE; // e.g., 32
        
        ifb_we_ext    = 0;
        ifb_waddr_ext = 0;
        ifb_wdata_ext = 0;
        
        wb_we_ext     = 0;
        wb_waddr_ext  = 0;
        wb_wdata_ext  = 0;
        
        @(posedge rst_n);
        #10;
        
        // ------------------------------------------------------------------------
        // 步骤 1: 从 mem 加载数据到 IFB
        // ------------------------------------------------------------------------
        $display("-------------------------------------------");
        $display("Step 1: Loading input feature map to IFB...");
        for (int i = 0; i < (H_IN * W_IN); i++) begin
            @(posedge clk);
            ifb_we_ext    <= 1'b1;
            ifb_waddr_ext <= i;
            ifb_wdata_ext <= ifm_mem[i];
        end
        @(posedge clk);
        ifb_we_ext <= 1'b0;
        
        // ------------------------------------------------------------------------
        // 步骤 2: 加载权重到 WB
        // ------------------------------------------------------------------------
        $display("Step 2: Loading weights to WB...");
        for (int w = 0; w < (K * K); w++) begin
            @(posedge clk);
            wb_we_ext    <= 1'b1;
            wb_waddr_ext <= w;
            wb_wdata_ext <= weight_mem[w];
        end
        @(posedge clk);
        wb_we_ext <= 1'b0;
        
        // ------------------------------------------------------------------------
        // 步骤 3: 启动加速器进行数据流计算
        // ------------------------------------------------------------------------
        $display("Step 3: Starting accelerator computation...");
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
        
        // 等待计算完成信号
        wait(done == 1'b1);
        $display("Accelerator computation finished!");
        
        // ------------------------------------------------------------------------
        // 步骤 4: 自动比对验证
        // ------------------------------------------------------------------------
        #20;
        begin
            int fd;
            int err_cnt;
            err_cnt = 0;
            fd = $fopen("sim_result.txt", "w");
            
            $display("-------------------------------------------");
            $display("Automated Result Verification (OFM against Expected)");
            $fdisplay(fd, "-------------------------------------------");
            $fdisplay(fd, "Automated Result Verification (OFM against Expected)");
            
            // Wait for the pipeline to drain (write back takes 3 cycles)
            #30;
            
            // Validate all computed pixels
            for (int p = 0; p < NUM_PIXELS_TO_COMPUTE; p++) begin
                logic [255:0] expected_pixel;
                expected_pixel = ofm_expected[p];
                for (int c = 0; c < NUM_COL; c++) begin
                    logic signed [PSUM_WIDTH-1:0] expected_psum;
                    logic signed [PSUM_WIDTH-1:0] actual_psum;
                    
                    // Extract corresponding 32-bit expected output for channel c
                    expected_psum = expected_pixel[c*PSUM_WIDTH +: PSUM_WIDTH];
                    
                    case (c)
                        0: actual_psum = dut.u_mac_array.gen_col[0].u_col.parf_ram[p];
                        1: actual_psum = dut.u_mac_array.gen_col[1].u_col.parf_ram[p];
                        2: actual_psum = dut.u_mac_array.gen_col[2].u_col.parf_ram[p];
                        3: actual_psum = dut.u_mac_array.gen_col[3].u_col.parf_ram[p];
                        4: actual_psum = dut.u_mac_array.gen_col[4].u_col.parf_ram[p];
                        5: actual_psum = dut.u_mac_array.gen_col[5].u_col.parf_ram[p];
                        6: actual_psum = dut.u_mac_array.gen_col[6].u_col.parf_ram[p];
                        7: actual_psum = dut.u_mac_array.gen_col[7].u_col.parf_ram[p];
                        default: actual_psum = '0;
                    endcase
                    
                    if (actual_psum !== expected_psum) begin
                        $error("Mismatch at Pixel %0d, Channel %0d! Expected = %0d, Actual = %0d", p, c, expected_psum, actual_psum);
                        $fdisplay(fd, "ERROR: Mismatch at Pixel %0d, Channel %0d! Expected = %0d, Actual = %0d", p, c, expected_psum, actual_psum);
                        err_cnt++;
                    end
                end
            end
            
            if (err_cnt == 0) begin
                $display("SUCCESS: All %0d pixels matched the Python reference theoretical values!", NUM_PIXELS_TO_COMPUTE);
                $fdisplay(fd, "SUCCESS: All %0d pixels matched the Python reference theoretical values!", NUM_PIXELS_TO_COMPUTE);
            end else begin
                $display("FAILED: Found %0d mismatches.", err_cnt);
                $fdisplay(fd, "FAILED: Found %0d mismatches.", err_cnt);
            end
            
            $display("===============================================================");
            $display("                    PERFORMANCE REPORT                         ");
            $display("===============================================================");
            $display("| Component  | Read Acc | Write Acc | Total Data Vol (Bytes)  |");
            $display("---------------------------------------------------------------");
            $display("| SRAM (IFB) | %8d | %9d | %23d |", cnt_ifb_rd, cnt_ifb_wr, (cnt_ifb_rd + cnt_ifb_wr) * 8);
            $display("| SRAM (WB)  | %8d | %9d | %23d |", cnt_wb_rd,  cnt_wb_wr,  (cnt_wb_rd + cnt_wb_wr) * 64);
            $display("| RF (ARF)   | %8d | %9d | %23d |", cnt_arf_rd, cnt_arf_wr, (cnt_arf_rd + cnt_arf_wr) * 8);
            $display("| RF (WRF)   | %8d | %9d | %23d |", cnt_wrf_rd, cnt_wrf_wr, (cnt_wrf_rd + cnt_wrf_wr) * 64);
            $display("| RF (PARF)  | %8d | %9d | %23d |", cnt_parf_rd, cnt_parf_wr, (cnt_parf_rd + cnt_parf_wr) * 32);
            $display("===============================================================");
            
            $fdisplay(fd, "===============================================================");
            $fdisplay(fd, "                    PERFORMANCE REPORT                         ");
            $fdisplay(fd, "===============================================================");
            $fdisplay(fd, "| Component  | Read Acc | Write Acc | Total Data Vol (Bytes)  |");
            $fdisplay(fd, "---------------------------------------------------------------");
            $fdisplay(fd, "| SRAM (IFB) | %8d | %9d | %23d |", cnt_ifb_rd, cnt_ifb_wr, (cnt_ifb_rd + cnt_ifb_wr) * 8);
            $fdisplay(fd, "| SRAM (WB)  | %8d | %9d | %23d |", cnt_wb_rd,  cnt_wb_wr,  (cnt_wb_rd + cnt_wb_wr) * 64);
            $fdisplay(fd, "| RF (ARF)   | %8d | %9d | %23d |", cnt_arf_rd, cnt_arf_wr, (cnt_arf_rd + cnt_arf_wr) * 8);
            $fdisplay(fd, "| RF (WRF)   | %8d | %9d | %23d |", cnt_wrf_rd, cnt_wrf_wr, (cnt_wrf_rd + cnt_wrf_wr) * 64);
            $fdisplay(fd, "| RF (PARF)  | %8d | %9d | %23d |", cnt_parf_rd, cnt_parf_wr, (cnt_parf_rd + cnt_parf_wr) * 32);
            $fdisplay(fd, "===============================================================");
            
            $display("-------------------------------------------");
            $display("Test Finished!");
            $fdisplay(fd, "-------------------------------------------");
            $fdisplay(fd, "Test Finished!");
            
            $fclose(fd);
            $finish;
        end
    end

endmodule
