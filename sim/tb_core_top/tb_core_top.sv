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

    //=============================================================================
    // 2. 激励加载和流程控制
    //=============================================================================
    initial begin
        // 初始化外部信号
        start         = 0;
        kernel_size   = 4'd9;   // 3x3=9 个位置的权重
        num_pixels    = 8'd32;  // 一次处理 32 个空间像素
        
        ifb_we_ext    = 0;
        ifb_waddr_ext = 0;
        ifb_wdata_ext = 0;
        
        wb_we_ext     = 0;
        wb_waddr_ext  = 0;
        wb_wdata_ext  = 0;
        
        @(posedge rst_n);
        #10;
        
        // ------------------------------------------------------------------------
        // 步骤 1: 加载合成数据到 IFB
        // 为了方便人为核对结果：假设每个像素的8个通道都等于它的位置索引 (例如位置0通道值全为1，位置1通道值全为2)
        // 实际数据宽为 64bit: 8个8bit。
        // ------------------------------------------------------------------------
        $display("-------------------------------------------");
        $display("Step 1: Loading input feature map to IFB...");
        for (int i = 0; i < 32; i++) begin
            @(posedge clk);
            ifb_we_ext    <= 1'b1;
            ifb_waddr_ext <= i;
            // 填入简单递增数: 第i个像素全通道为i+1
            ifb_wdata_ext <= {8{ 8'(i+1) }};
        end
        @(posedge clk);
        ifb_we_ext <= 1'b0;
        
        // ------------------------------------------------------------------------
        // 步骤 2: 加载合成数据到 WB
        // 假设 3x3=9 组权重，每组权重包含 8x8=64 个值
        // 为方便验证：假设全部权重都等于 1
        // ------------------------------------------------------------------------
        $display("Step 2: Loading weights to WB...");
        for (int w = 0; w < 9; w++) begin
            @(posedge clk);
            wb_we_ext    <= 1'b1;
            wb_waddr_ext <= w;
            wb_wdata_ext <= {64{8'sd1}}; // 64个8-bit全部为正数1
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
        $display("Debug: parf_ram[31] of col 0 = %0d", dut.u_mac_array.gen_col[0].u_col.parf_ram[31]);
        $display("Debug: prod_array[0] in col 0 = %0d", dut.u_mac_array.gen_col[0].u_col.prod_array[0]);
        $display("Debug: act_out_vec from ARF = %h", dut.u_arf.act_out_vec);
        $display("Debug: active_weight in PE0 = %0d", dut.u_mac_array.gen_col[0].u_col.gen_pe[0].u_pe.active_weight);
        
        // ------------------------------------------------------------------------
        // 步骤 4: 人工验证结果
        // 预期结果计算：
        // 对于像素 i，通道值均为 (i+1)。权重全为 1。
        // 每一列(输出通道)有8个PE，相乘累加：8 * (i+1) * 1 = 8*(i+1)。
        // 共累加 9 次(9轮权重)：9 * 8*(i+1) = 72 * (i+1)。
        // 验证最后一个周期的 psum_out_vec，应该是像素 31 (i=31) 的结果：
        // 预期 = 72 * 32 = 2304 = 0x0900
        // ------------------------------------------------------------------------
        #20;
        begin
            int fd;
            fd = $fopen("sim_result.txt", "w");
            
            $display("-------------------------------------------");
            $display("Final Psum Output Vector (at PARF Address 31):");
            $fdisplay(fd, "-------------------------------------------");
            $fdisplay(fd, "Final Psum Output Vector (at PARF Address 31):");
            
            // Wait for the pipeline to drain (write back takes 3 cycles)
            #30;
            
            for (int c = 0; c < NUM_COL; c++) begin
                logic signed [PSUM_WIDTH-1:0] single_psum;
                case (c)
                    0: single_psum = dut.u_mac_array.gen_col[0].u_col.parf_ram[31];
                    1: single_psum = dut.u_mac_array.gen_col[1].u_col.parf_ram[31];
                    2: single_psum = dut.u_mac_array.gen_col[2].u_col.parf_ram[31];
                    3: single_psum = dut.u_mac_array.gen_col[3].u_col.parf_ram[31];
                    4: single_psum = dut.u_mac_array.gen_col[4].u_col.parf_ram[31];
                    5: single_psum = dut.u_mac_array.gen_col[5].u_col.parf_ram[31];
                    6: single_psum = dut.u_mac_array.gen_col[6].u_col.parf_ram[31];
                    7: single_psum = dut.u_mac_array.gen_col[7].u_col.parf_ram[31];
                    default: single_psum = '0;
                endcase
                $display("Output Channel %0d: Expected = 2304, Actual = %0d", c, single_psum);
                $fdisplay(fd, "Output Channel %0d: Expected = 2304, Actual = %0d", c, single_psum);
                if (single_psum !== 32'sd2304) begin
                    $error("Mismatch at channel %0d!", c);
                    $fdisplay(fd, "ERROR: Mismatch at channel %0d!", c);
                end else begin
                    $fdisplay(fd, "SUCCESS: Channel %0d matched expected value.", c);
                end
            end
            
            $display("-------------------------------------------");
            $display("Test Finished!");
            $fdisplay(fd, "-------------------------------------------");
            $fdisplay(fd, "Test Finished!");
            
            $fclose(fd);
            $finish;
        end
    end

endmodule
