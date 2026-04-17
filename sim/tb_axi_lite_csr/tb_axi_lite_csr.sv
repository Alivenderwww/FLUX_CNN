`timescale 1ns/1ps

// =============================================================================
// tb_axi_lite_csr.sv  --  smoke test: AXI-Lite S 写 16 个寄存器，再读回验证
// =============================================================================
module tb_axi_lite_csr;

    localparam int ADDR_W = 12;
    localparam int DATA_W = 32;
    localparam int N      = 16;

    logic clk  = 0;
    logic rstn = 0;
    always #5 clk = ~clk;

    // AXI-Lite signals
    logic [ADDR_W-1:0]    AWADDR;
    logic                 AWVALID;
    logic                 AWREADY;
    logic [DATA_W-1:0]    WDATA;
    logic [DATA_W/8-1:0]  WSTRB;
    logic                 WVALID;
    logic                 WREADY;
    logic [1:0]           BRESP;
    logic                 BVALID;
    logic                 BREADY;
    logic [ADDR_W-1:0]    ARADDR;
    logic                 ARVALID;
    logic                 ARREADY;
    logic [DATA_W-1:0]    RDATA;
    logic [1:0]           RRESP;
    logic                 RVALID;
    logic                 RREADY;

    // Back-end register file (simple N-reg bank)
    logic                 reg_w_en;
    logic [ADDR_W-1:0]    reg_w_addr;
    logic [DATA_W-1:0]    reg_w_data;
    logic [DATA_W/8-1:0]  reg_w_strb;
    logic [ADDR_W-1:0]    reg_r_addr;
    logic [DATA_W-1:0]    reg_r_data;

    logic [DATA_W-1:0] regs [0:N-1];
    always_ff @(posedge clk) begin
        if (!rstn) begin
            for (int i = 0; i < N; i++) regs[i] <= '0;
        end else if (reg_w_en) begin
            regs[reg_w_addr[$clog2(N)+1 : 2]] <= reg_w_data;  // 按 4 字节对齐取 bank 索引
        end
    end
    assign reg_r_data = regs[reg_r_addr[$clog2(N)+1 : 2]];

    // DUT
    axi_lite_csr #(
        .ADDR_W(ADDR_W),
        .DATA_W(DATA_W)
    ) dut (
        .clk(clk), .rstn(rstn),
        .AWADDR(AWADDR), .AWVALID(AWVALID), .AWREADY(AWREADY),
        .WDATA(WDATA), .WSTRB(WSTRB), .WVALID(WVALID), .WREADY(WREADY),
        .BRESP(BRESP), .BVALID(BVALID), .BREADY(BREADY),
        .ARADDR(ARADDR), .ARVALID(ARVALID), .ARREADY(ARREADY),
        .RDATA(RDATA), .RRESP(RRESP), .RVALID(RVALID), .RREADY(RREADY),
        .reg_w_en(reg_w_en), .reg_w_addr(reg_w_addr), .reg_w_data(reg_w_data), .reg_w_strb(reg_w_strb),
        .reg_r_addr(reg_r_addr), .reg_r_data(reg_r_data)
    );

    // AXI-Lite drive tasks
    //   关键点：valid/ready 的驱动用 nonblocking `<=`，把 deassert 推迟到 NBA
    //   之后，避免与 DUT 的 always_ff 在同拍 Active 区发生读写 race。
    task automatic axi_lite_write(input [ADDR_W-1:0] addr, input [DATA_W-1:0] data);
        AWADDR  <= addr;
        AWVALID <= 1'b1;
        WDATA   <= data;
        WSTRB   <= '1;
        WVALID  <= 1'b1;
        do @(posedge clk); while (!(AWVALID && AWREADY));
        AWVALID <= 1'b0;
        while (!(WVALID && WREADY)) @(posedge clk);
        WVALID  <= 1'b0;

        BREADY <= 1'b1;
        do @(posedge clk); while (!(BVALID && BREADY));
        BREADY <= 1'b0;
    endtask

    task automatic axi_lite_read(input [ADDR_W-1:0] addr, output [DATA_W-1:0] data);
        ARADDR  <= addr;
        ARVALID <= 1'b1;
        do @(posedge clk); while (!(ARVALID && ARREADY));
        ARVALID <= 1'b0;

        RREADY <= 1'b1;
        do @(posedge clk); while (!(RVALID && RREADY));
        data = RDATA;
        RREADY <= 1'b0;
    endtask

    int mismatch_cnt = 0;
    logic [DATA_W-1:0] readback;

    initial begin
        AWADDR = 0; AWVALID = 0; WDATA = 0; WSTRB = 0; WVALID = 0; BREADY = 0;
        ARADDR = 0; ARVALID = 0; RREADY = 0;

        #20 rstn = 1;
        #10;

        $display("=== Phase 1: write 16 regs ===");
        for (int i = 0; i < N; i++) begin
            axi_lite_write(i * 4, 32'hDEAD_0000 + i);
        end

        #20;
        $display("=== Phase 2: read back 16 regs ===");
        for (int i = 0; i < N; i++) begin
            axi_lite_read(i * 4, readback);
            if (readback !== 32'hDEAD_0000 + i) begin
                $display("  FAIL: reg[%0d] expect=%08h got=%08h",
                         i, 32'hDEAD_0000 + i, readback);
                mismatch_cnt++;
            end
        end

        if (mismatch_cnt == 0) $display("SMOKE TEST PASSED. 0 mismatches.");
        else                    $display("SMOKE TEST FAILED. %0d mismatches.", mismatch_cnt);
        $stop;
    end

    initial begin
        #100000;
        $display("FATAL: timeout");
        $stop;
    end

endmodule
