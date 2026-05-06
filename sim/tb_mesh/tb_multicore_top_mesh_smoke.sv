`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// tb_multicore_top_mesh_smoke.sv  --  multicore_top_mesh.sv 接线 smoke test
//
// 仅验证:
//   1. multicore_top_mesh 跟所有依赖能 elab + reset 正常起来
//   2. 4 ConvCore + 4 mem_core + mesh 互连无 X 信号 / 无锁死
//   3. mem_core[0] backdoor 推一个 packet 到 ConvCore[0], 检查 ConvCore[0] IFB SRAM
//
// 不跑 conv 计算 (那是 Step E-F). 只验证连线/复位/数据流向正确.
// =============================================================================

module tb_multicore_top_mesh_smoke;
    timeunit 1ns; timeprecision 1ps;

    localparam int NUM_CORES = 4;
    localparam int CSR_DATA_W = `FLUX_CSR_DATA_W;
    localparam int BUS_ADDR_W = `FLUX_BUS_ADDR_W;
    localparam int BUS_DATA_W = `FLUX_BUS_DATA_W;
    localparam int AXI_M_ID    = `FLUX_AXI_M_ID;
    localparam int AXI_M_W     = `FLUX_AXI_M_WIDTH;
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_W;
    localparam int EXT_BUS_ID  = CORE_BUS_ID + 2;       // CORE_ID_W=2
    localparam int CORE_ID_W   = 2;
    localparam int HOST_CSR_AW = 12 + CORE_ID_W;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ---- DUT 信号 ----
    logic [HOST_CSR_AW-1:0]  csr_awaddr  = '0;
    logic                    csr_awvalid = 0, csr_awready;
    logic [CSR_DATA_W-1:0]   csr_wdata   = '0;
    logic [CSR_DATA_W/8-1:0] csr_wstrb   = '0;
    logic                    csr_wvalid  = 0, csr_wready;
    logic [1:0]              csr_bresp;
    logic                    csr_bvalid, csr_bready = 1'b1;
    logic [HOST_CSR_AW-1:0]  csr_araddr  = '0;
    logic                    csr_arvalid = 0, csr_arready;
    logic [CSR_DATA_W-1:0]   csr_rdata;
    logic [1:0]              csr_rresp;
    logic                    csr_rvalid, csr_rready = 1'b1;

    // mem CSR
    logic [HOST_CSR_AW-1:0]  mem_csr_awaddr  = '0;
    logic                    mem_csr_awvalid = 0, mem_csr_awready;
    logic [CSR_DATA_W-1:0]   mem_csr_wdata   = '0;
    logic [CSR_DATA_W/8-1:0] mem_csr_wstrb   = '0;
    logic                    mem_csr_wvalid  = 0, mem_csr_wready;
    logic [1:0]              mem_csr_bresp;
    logic                    mem_csr_bvalid, mem_csr_bready = 1'b1;
    logic [HOST_CSR_AW-1:0]  mem_csr_araddr  = '0;
    logic                    mem_csr_arvalid = 0, mem_csr_arready;
    logic [CSR_DATA_W-1:0]   mem_csr_rdata;
    logic [1:0]              mem_csr_rresp;
    logic                    mem_csr_rvalid, mem_csr_rready = 1'b1;

    logic [EXT_BUS_ID-1:0]   bus_arid;
    logic [BUS_ADDR_W-1:0]   bus_araddr;
    logic [7:0]              bus_arlen;
    logic [2:0]              bus_arsize;
    logic [1:0]              bus_arburst;
    logic                    bus_arlock;
    logic [3:0]              bus_arcache;
    logic [2:0]              bus_arprot;
    logic [3:0]              bus_arqos;
    logic                    bus_arvalid;
    logic                    bus_arready = 1'b1;       // 永远 ready (read 不用)
    logic [EXT_BUS_ID-1:0]   bus_rid     = '0;
    logic [BUS_DATA_W-1:0]   bus_rdata   = '0;
    logic [1:0]              bus_rresp   = '0;
    logic                    bus_rlast   = 1'b0;
    logic                    bus_rvalid  = 1'b0;
    logic                    bus_rready;

    logic [NUM_CORES-1:0]    done_per_core;

    multicore_top_mesh u_dut (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb),
        .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp),
        .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),
        .mem_csr_awaddr(mem_csr_awaddr), .mem_csr_awvalid(mem_csr_awvalid), .mem_csr_awready(mem_csr_awready),
        .mem_csr_wdata(mem_csr_wdata), .mem_csr_wstrb(mem_csr_wstrb),
        .mem_csr_wvalid(mem_csr_wvalid), .mem_csr_wready(mem_csr_wready),
        .mem_csr_bresp(mem_csr_bresp), .mem_csr_bvalid(mem_csr_bvalid), .mem_csr_bready(mem_csr_bready),
        .mem_csr_araddr(mem_csr_araddr), .mem_csr_arvalid(mem_csr_arvalid), .mem_csr_arready(mem_csr_arready),
        .mem_csr_rdata(mem_csr_rdata), .mem_csr_rresp(mem_csr_rresp),
        .mem_csr_rvalid(mem_csr_rvalid), .mem_csr_rready(mem_csr_rready),
        .bus_arid(bus_arid), .bus_araddr(bus_araddr),
        .bus_arlen(bus_arlen), .bus_arsize(bus_arsize),
        .bus_arburst(bus_arburst), .bus_arlock(bus_arlock),
        .bus_arcache(bus_arcache), .bus_arprot(bus_arprot),
        .bus_arqos(bus_arqos),
        .bus_arvalid(bus_arvalid), .bus_arready(bus_arready),
        .bus_rid(bus_rid), .bus_rdata(bus_rdata), .bus_rresp(bus_rresp),
        .bus_rlast(bus_rlast), .bus_rvalid(bus_rvalid), .bus_rready(bus_rready),
        .done_per_core(done_per_core)
    );

    // ----- 主流程 -----
    int errors = 0;

    // axi-lite write helper (写到 ConvCore[i] 的 reg_addr)
    task automatic host_write(input int core_id, input logic [11:0] reg_addr, input logic [31:0] data);
        @(posedge clk);
        csr_awaddr  <= {2'(core_id), reg_addr};
        csr_awvalid <= 1'b1;
        csr_wdata   <= data;
        csr_wstrb   <= '1;
        csr_wvalid  <= 1'b1;
        do @(posedge clk); while (!(csr_awvalid && csr_awready));
        csr_awvalid <= 1'b0;
        while (!(csr_wvalid && csr_wready)) @(posedge clk);
        csr_wvalid  <= 1'b0;
        do @(posedge clk); while (!(csr_bvalid && csr_bready));
    endtask

    // mem CSR 写
    task automatic mem_write(input int mem_id, input logic [11:0] reg_addr, input logic [31:0] data);
        @(posedge clk);
        mem_csr_awaddr  <= {2'(mem_id), reg_addr};
        mem_csr_awvalid <= 1'b1;
        mem_csr_wdata   <= data;
        mem_csr_wstrb   <= '1;
        mem_csr_wvalid  <= 1'b1;
        do @(posedge clk); while (!(mem_csr_awvalid && mem_csr_awready));
        mem_csr_awvalid <= 1'b0;
        while (!(mem_csr_wvalid && mem_csr_wready)) @(posedge clk);
        mem_csr_wvalid  <= 1'b0;
        do @(posedge clk); while (!(mem_csr_bvalid && mem_csr_bready));
    endtask

    // mem CSR 读
    task automatic mem_read(input int mem_id, input logic [11:0] reg_addr, output logic [31:0] data);
        @(posedge clk);
        mem_csr_araddr  <= {2'(mem_id), reg_addr};
        mem_csr_arvalid <= 1'b1;
        do @(posedge clk); while (!(mem_csr_arvalid && mem_csr_arready));
        mem_csr_arvalid <= 1'b0;
        do @(posedge clk); while (!(mem_csr_rvalid));
        data = mem_csr_rdata;
    endtask

    // DEBUG: monitor mesh fire on Mem[0] tx → ConvCore[0] rx
    int mem0_tx_fire = 0;
    int conv0_rx_fire = 0;
    always @(posedge clk) begin
        if (rst_n) begin
            // Mem[0].m_axis = mesh_in[0]
            if (u_dut.u_mesh.s_axis_local_tvalid[0] && u_dut.u_mesh.s_axis_local_tready[0])
                mem0_tx_fire <= mem0_tx_fire + 1;
            // ConvCore[0] rx 端 = mesh_out_local[4]
            if (u_dut.u_mesh.m_axis_local_tvalid[4] && u_dut.u_mesh.m_axis_local_tready[4])
                conv0_rx_fire <= conv0_rx_fire + 1;
        end
    end

    initial begin
        #20 rst_n = 1;
        #10;

        $display("== tb_multicore_top_mesh_smoke: connectivity + reset + packet 1 ==");

        // T1: reset 后所有 done=0
        repeat (5) @(posedge clk);
        if (done_per_core != 4'd0) begin
            $display("  FAIL T1: done_per_core=%b expect 0", done_per_core);
            errors++;
        end else begin
            $display("  PASS T1: done_per_core=0 after reset");
        end

        // T2: 通过 host CSR 给 ConvCore[0] 配 SKIP_IDMA + IFB ring
        //   (SKIP_IDMA=0 → ifb_axi_slave 拒收 push; ring_words=0 → ring 反压死锁)
        host_write(0, `FLUX_ADDR_SKIP_IDMA,      32'd1);
        host_write(0, `FLUX_ADDR_IFB_STRIP_ROWS, 32'd64);
        host_write(0, `FLUX_ADDR_IFB_RING_WORDS, 32'd1024);
        $display("  [host] ConvCore[0]: SKIP_IDMA=1 strip_rows=64 ring_words=1024");

        // line_buffer.rows_consumed_raw 没 reset 默认 X (smoke 不启动 layer);
        //   实际 deployment evt_start_layer 拉时 line_buffer 内部 reset.
        //   smoke 单测留一处 force 处理 X 传播问题.
        force u_dut.gen_core[0].u_conv.u_core.u_line_buffer.rows_consumed_raw = 16'd0;
        @(posedge clk);

        // T2b: backdoor 把数据 preload 到 mem_core[0].ddr_mem (= IFB 数据源)
        for (int i = 0; i < 16; i++)
            u_dut.gen_mem[0].u_mem.ddr_mem[i] = {32'h0, 32'h0, 32'd0, 32'(i)};

        // T3: 通过 mem AXI-Lite CSR 配 + trigger mem_core[0] 发 1 个 packet 给 ConvCore[0]
        //   ConvCore[0] @ (0, 1) 即 tdest = {y=1, x=0} = 8'h10
        mem_write(0, 12'h000, 32'h0000_0000);             // CMD_DDR_ADDR
        mem_write(0, 12'h004, 32'd16);                    // CMD_BURST_LEN
        mem_write(0, 12'h008, 32'h0000_0000);             // CMD_SRAM_OFFSET (ConvCore IFB[0])
        mem_write(0, 12'h00C, 32'h0000_1000);             // tdest=8'h10, opcode=0
        mem_write(0, 12'h010, 32'h0000_0001);             // TRIGGER
        // poll done sticky
        begin
            logic [31:0] status;
            int poll_cnt = 0;
            do begin
                mem_read(0, 12'h014, status);
                poll_cnt++;
                if (poll_cnt > 200) begin
                    $display("  TIMEOUT: mem cmd_done sticky never set"); errors++; break;
                end
            end while (!status[1]);
        end

        // T4: 检查 ConvCore[0] 内部 IFB SRAM[0..15]
        // hier ref: u_dut.gen_core[0].u_conv.u_core.u_ifb.mem[i]
        $display("\n[T4] Check ConvCore[0] IFB SRAM[0..15]:");
        begin
            automatic int mismatch = 0;
            for (int i = 0; i < 16; i++) begin
                automatic logic [BUS_DATA_W-1:0] got = u_dut.gen_core[0].u_conv.u_core.u_ifb.mem[i];
                automatic logic [BUS_DATA_W-1:0] exp = {32'h0, 32'h0, 32'd0, 32'(i)};
                if (got !== exp) begin
                    if (mismatch < 3)
                        $display("  FAIL: ifb[%0d]=%h expect %h", i, got, exp);
                    mismatch++;
                end
            end
            if (mismatch == 0)
                $display("  PASS T4: ConvCore[0] IFB SRAM[0..15] all match");
            else
                errors += mismatch;
        end

        $display("\n[DEBUG] mesh fire counters:");
        $display("  Mem[0] tx fire (mesh in[0]):  %0d", mem0_tx_fire);
        $display("  Conv[0] rx fire (mesh out[4]): %0d", conv0_rx_fire);
        $display("  cfg_skip_idma signal: %b",
                 u_dut.gen_core[0].u_conv.u_core.u_cfg.skip_idma);
        $display("  Mem[0] tx busy=%b done_pulse=%b done_sticky=%b",
                 u_dut.gen_mem[0].u_mem.cmd_busy,
                 u_dut.gen_mem[0].u_mem.cmd_done_pulse,
                 u_dut.gen_mem[0].u_mem.r_cmd_done_sticky);
        $display("  ConvCore[0] rx_bridge st=%0d r_addr=0x%h r_awlen=%0d",
                 u_dut.gen_core[0].u_conv.u_rx_bridge.st,
                 u_dut.gen_core[0].u_conv.u_rx_bridge.r_addr,
                 u_dut.gen_core[0].u_conv.u_rx_bridge.r_awlen);
        $display("  ifb_axi_slave wst=%0d rows_pushed=%0d",
                 u_dut.gen_core[0].u_conv.u_core.u_ifb_axi_slv.wst,
                 u_dut.gen_core[0].u_conv.u_core.u_ifb_axi_slv.rows_pushed_out);

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (multicore_top_mesh smoke - mesh path 通)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #50000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
