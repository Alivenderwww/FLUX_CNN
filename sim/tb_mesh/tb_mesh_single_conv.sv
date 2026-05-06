`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// tb_mesh_single_conv.sv  --  Single-core conv 走 mesh 路径 bit-exact
//
// 复刻 tb_core_dma 的 case01 流程, 但 DUT 用 multicore_top_mesh:
//   - 只用 ConvCore[0] (其他 3 核 SKIP_IDMA 但不启动)
//   - IFB 数据从 mem_core[0].ddr_mem 推 (mesh AXIS 路径)
//   - WB / RDMA / desc 仍走 axi_slave_mem (ConvCore.bus_ar/r read)
//   - ODMA 输出 → axi_writer_to_axis → mesh → mem_core[0].ddr_mem (OFB 区)
//   - bit-exact 比对 mem_core[0].ddr_mem 跟 cases/case01/expected_ofm.txt
//
// case01 cfg: K=3 stride=2 c_in=16 c_out=16 H=240 W=135 → H_out=120 W_out=68
// =============================================================================

module tb_mesh_single_conv;
    timeunit 1ns; timeprecision 1ps;

    // ============== 参数 ==============
    localparam int NUM_CORES   = 4;
    localparam int NUM_COL     = `FLUX_NUM_COL;
    localparam int NUM_PE      = `FLUX_NUM_PE;
    localparam int DATA_WIDTH  = `FLUX_DATA_WIDTH;
    localparam int CSR_DATA_W  = `FLUX_CSR_DATA_W;
    localparam int BUS_ADDR_W  = `FLUX_BUS_ADDR_W;
    localparam int BUS_DATA_W  = `FLUX_BUS_DATA_W;
    localparam int AXI_M_ID    = `FLUX_AXI_M_ID;
    localparam int AXI_M_W     = `FLUX_AXI_M_WIDTH;
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_W;
    localparam int CORE_ID_W   = 2;
    localparam int HOST_CSR_AW = 12 + CORE_ID_W;
    localparam int EXT_BUS_ID  = CORE_BUS_ID + CORE_ID_W;
    localparam int IFB_WIDTH   = NUM_PE * DATA_WIDTH;        // 128
    localparam int OFB_WIDTH   = NUM_COL * DATA_WIDTH;       // 128
    localparam int WB_WIDTH    = NUM_COL * NUM_PE * DATA_WIDTH; // 2048

    // DDR 布局 (跟 tb_core_dma 同, 但 IFB 不进 axi_slave_mem 而是 mem_core)
    localparam [31:0] DDR_IFB_BASE  = 32'h0000_0000;        // mem_core[0].ddr_mem 内
    localparam [31:0] DDR_DESC_BASE = 32'h007F_0000;        // axi_slave_mem 内
    localparam [31:0] DDR_WB_BASE   = 32'h0080_0000;
    localparam [31:0] DDR_OFB_BASE  = 32'h0090_0000;        // OFM 写回 mem_core[0].ddr_mem
    localparam [31:0] DDR_RDMA_BASE = 32'h00A0_0000;

    // cfg_regs 关键 addr (从 tb_core_dma 复制)
    localparam [11:0] ADDR_CTRL           = `FLUX_ADDR_CTRL;
    localparam [11:0] ADDR_DESC_LIST_BASE = `FLUX_ADDR_DESC_LIST_BASE;
    localparam [11:0] ADDR_DESC_COUNT     = `FLUX_ADDR_DESC_COUNT;
    localparam [11:0] ADDR_DMA_MODE       = 12'h17C;
    localparam [11:0] ADDR_SKIP_IDMA      = `FLUX_ADDR_SKIP_IDMA;
    localparam [11:0] ADDR_IFB_STRIP_ROWS = `FLUX_ADDR_IFB_STRIP_ROWS;
    localparam [11:0] ADDR_IFB_RING_WORDS = `FLUX_ADDR_IFB_RING_WORDS;
    localparam [11:0] ADDR_OFM_TDEST      = `FLUX_ADDR_OFM_TDEST;
    localparam [11:0] ADDR_OFM_OPCODE     = `FLUX_ADDR_OFM_OPCODE;

    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ============== DUT 信号 (ConvCore CSR) ==============
    logic [HOST_CSR_AW-1:0] csr_awaddr;
    logic                   csr_awvalid, csr_awready;
    logic [CSR_DATA_W-1:0]  csr_wdata;
    logic [CSR_DATA_W/8-1:0]csr_wstrb;
    logic                   csr_wvalid, csr_wready;
    logic [1:0]             csr_bresp;
    logic                   csr_bvalid, csr_bready;
    logic [HOST_CSR_AW-1:0] csr_araddr  = 0;
    logic                   csr_arvalid = 0, csr_arready;
    logic [CSR_DATA_W-1:0]  csr_rdata;
    logic [1:0]             csr_rresp;
    logic                   csr_rvalid, csr_rready = 0;

    // mem CSR
    logic [HOST_CSR_AW-1:0] mem_csr_awaddr;
    logic                   mem_csr_awvalid, mem_csr_awready;
    logic [CSR_DATA_W-1:0]  mem_csr_wdata;
    logic [CSR_DATA_W/8-1:0]mem_csr_wstrb;
    logic                   mem_csr_wvalid, mem_csr_wready;
    logic [1:0]             mem_csr_bresp;
    logic                   mem_csr_bvalid, mem_csr_bready;
    logic [HOST_CSR_AW-1:0] mem_csr_araddr  = 0;
    logic                   mem_csr_arvalid = 0, mem_csr_arready;
    logic [CSR_DATA_W-1:0]  mem_csr_rdata;
    logic [1:0]             mem_csr_rresp;
    logic                   mem_csr_rvalid, mem_csr_rready = 1'b1;

    // axi master (read only) 接 axi_slave_mem
    logic [EXT_BUS_ID-1:0]  bus_arid;
    logic [BUS_ADDR_W-1:0]  bus_araddr;
    logic [7:0]             bus_arlen;
    logic [2:0]             bus_arsize;
    logic [1:0]             bus_arburst;
    logic                   bus_arlock;
    logic [3:0]             bus_arcache;
    logic [2:0]             bus_arprot;
    logic [3:0]             bus_arqos;
    logic                   bus_arvalid, bus_arready;
    logic [EXT_BUS_ID-1:0]  bus_rid;
    logic [BUS_DATA_W-1:0]  bus_rdata;
    logic [1:0]             bus_rresp;
    logic                   bus_rlast, bus_rvalid;
    logic                   bus_rready;

    logic [NUM_CORES-1:0]   done_per_core;

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

    // axi_slave_mem 服务 ConvCore.bus_ar/r (WB / RDMA / desc fetch)
    // write 通道 tie 0 (ConvCore.bus_aw/w/b 不接, mesh 模式不用)
    logic [EXT_BUS_ID-1:0]  bus_awid_tie  = 0;
    logic [BUS_ADDR_W-1:0]  bus_awaddr_tie = 0;
    logic [7:0]             bus_awlen_tie = 0;
    logic [1:0]             bus_awburst_tie = 0;
    logic                   bus_awvalid_tie = 0, bus_awready_tie;
    logic [BUS_DATA_W-1:0]  bus_wdata_tie  = 0;
    logic [BUS_DATA_W/8-1:0]bus_wstrb_tie  = 0;
    logic                   bus_wlast_tie  = 0;
    logic                   bus_wvalid_tie = 0, bus_wready_tie;
    logic [EXT_BUS_ID-1:0]  bus_bid_tie;
    logic [1:0]             bus_bresp_tie;
    logic                   bus_bvalid_tie, bus_bready_tie = 1'b1;

    axi_slave_mem #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .ID_W(EXT_BUS_ID),
        .DEPTH(1048576)
    ) u_ddr (
        .clk(clk), .rstn(rst_n),
        .AWID(bus_awid_tie), .AWADDR(bus_awaddr_tie),
        .AWLEN(bus_awlen_tie), .AWBURST(bus_awburst_tie),
        .AWVALID(bus_awvalid_tie), .AWREADY(bus_awready_tie),
        .WDATA(bus_wdata_tie), .WSTRB(bus_wstrb_tie), .WLAST(bus_wlast_tie),
        .WVALID(bus_wvalid_tie), .WREADY(bus_wready_tie),
        .BID(bus_bid_tie), .BRESP(bus_bresp_tie),
        .BVALID(bus_bvalid_tie), .BREADY(bus_bready_tie),
        .ARID(bus_arid), .ARADDR(bus_araddr),
        .ARLEN(bus_arlen), .ARBURST(bus_arburst),
        .ARVALID(bus_arvalid), .ARREADY(bus_arready),
        .RID(bus_rid), .RDATA(bus_rdata), .RRESP(bus_rresp),
        .RLAST(bus_rlast), .RVALID(bus_rvalid), .RREADY(bus_rready)
    );

    // ============== Helpers ==============
    logic [BUS_DATA_W-1:0] ifb_arr  [0:524287];
    logic [WB_WIDTH-1:0]   wb_arr   [0:2047];
    logic [OFB_WIDTH-1:0]  exp_arr  [0:524287];
    logic [BUS_DATA_W-1:0] desc_arr [0:4095];
    logic [BUS_DATA_W-1:0] rdma_arr [0:8191];

    int    ifb_words_cfg, wb_words_cfg, ofb_words_cfg, rdma_words_cfg;
    int    h_out_cfg, w_out_cfg;
    string case_name_cfg;
    int    desc_count_cfg;

    task automatic axi_lite_write(input int core_id, input [11:0] reg_addr,
                                    input [31:0] data);
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
        csr_bready  <= 1'b1;
        do @(posedge clk); while (!(csr_bvalid && csr_bready));
        csr_bready  <= 1'b0;
    endtask

    // mem CSR 写 (核 id 选哪个 mem core)
    task automatic mem_axi_lite_write(input int mem_id, input [11:0] reg_addr,
                                       input [31:0] data);
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
        mem_csr_bready  <= 1'b1;
        do @(posedge clk); while (!(mem_csr_bvalid && mem_csr_bready));
        mem_csr_bready  <= 1'b0;
    endtask

    // mem CSR 读 (poll STATUS 等)
    task automatic mem_axi_lite_read(input int mem_id, input [11:0] reg_addr,
                                      output [31:0] data);
        @(posedge clk);
        mem_csr_araddr  <= {2'(mem_id), reg_addr};
        mem_csr_arvalid <= 1'b1;
        do @(posedge clk); while (!(mem_csr_arvalid && mem_csr_arready));
        mem_csr_arvalid <= 1'b0;
        do @(posedge clk); while (!(mem_csr_rvalid));
        data = mem_csr_rdata;
    endtask

    task automatic preload_ddr(input string case_dir);
        int ifb_base_w  = DDR_IFB_BASE  / 16;
        int wb_base_w   = DDR_WB_BASE   / 16;
        int desc_base_w = DDR_DESC_BASE / 16;
        int rdma_base_w = DDR_RDMA_BASE / 16;
        int desc_beats;

        // IFB 进 mem_core[0].ddr_mem (mesh 数据源)
        $readmemh($sformatf("%s/ifb.txt", case_dir), ifb_arr);
        for (int i = 0; i < ifb_words_cfg; i++)
            u_dut.gen_mem[0].u_mem.ddr_mem[ifb_base_w + i] = ifb_arr[i];

        // WB / desc / RDMA 进 axi_slave_mem (DDR mock)
        $readmemh($sformatf("%s/wb.txt",        case_dir), wb_arr);
        $readmemh($sformatf("%s/desc_list.hex", case_dir), desc_arr);
        if (rdma_words_cfg > 0) begin
            $readmemh($sformatf("%s/rdma_data.txt", case_dir), rdma_arr);
            for (int i = 0; i < rdma_words_cfg; i++)
                u_ddr.mem[rdma_base_w + i] = rdma_arr[i];
        end
        for (int i = 0; i < wb_words_cfg; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[wb_base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];

        desc_beats = 4096;
        for (int i = 0; i < desc_beats; i++) u_ddr.mem[desc_base_w + i] = desc_arr[i];

        // 清 mem_core[0].ddr_mem 的 OFB 区
        for (int i = 0; i < ofb_words_cfg; i++)
            u_dut.gen_mem[0].u_mem.ddr_mem[(DDR_OFB_BASE/16) + i] = '0;

        // 读 expected_ofm
        $readmemh($sformatf("%s/expected_ofm.txt", case_dir), exp_arr);
    endtask

    task automatic load_config(input string case_dir);
        int    fd, val, count;
        string line, key, path;
        path = $sformatf("%s/config.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("FATAL: cannot open %s", path); $stop; end
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            count = $sscanf(line, "%s = %d", key, val);
            if (count < 2) continue;
            case (key)
                "H_OUT"           : h_out_cfg = val;
                "W_OUT"           : w_out_cfg = val;
                "DMA_MODE"        : axi_lite_write(0, ADDR_DMA_MODE,     val);
                "DESC_COUNT"      : begin
                                       axi_lite_write(0, ADDR_DESC_COUNT, val);
                                       desc_count_cfg = val;
                                    end
                "_META_IFB_WORDS" : ifb_words_cfg = val;
                "_META_WB_WORDS"  : wb_words_cfg  = val;
                "_META_OFB_WORDS" : ofb_words_cfg = val;
                "_META_RDMA_WORDS": rdma_words_cfg = val;
                default: ;
            endcase
        end
        $fclose(fd);
    endtask

    // 推一个 IFB row packet via mem_core[0] AXI-Lite CSR
    //   mem CSR map (12-bit): 0x000=DDR_ADDR, 0x004=BURST_LEN, 0x008=SRAM_OFFSET,
    //                          0x00C={tdest[15:8], opcode[3:0]}, 0x010=TRIGGER, 0x014=STATUS
    task automatic mem_send_ifb_row(input int row_idx, input int row_words,
                                      input int sram_offset_words);
        logic [31:0] status;
        // stub 简化: SRAM_OFFSET 同时承担 mem 内部读起点 + packet header sram_addr,
        //   所以传 ddr_addr (mem 读 DDR 起点); 调用者保证 sram_offset_words == ddr_addr.
        int ddr_addr_w = (DDR_IFB_BASE/16) + row_idx * row_words;
        if (sram_offset_words != ddr_addr_w) begin
            $display("FATAL: stub 要求 sram_offset == ddr_addr, got %0d vs %0d",
                     sram_offset_words, ddr_addr_w);
            $stop;
        end
        mem_axi_lite_write(0, 12'h000, ddr_addr_w);                 // CMD_DDR_ADDR
        mem_axi_lite_write(0, 12'h004, row_words);                  // CMD_BURST_LEN
        mem_axi_lite_write(0, 12'h008, sram_offset_words);          // CMD_SRAM_OFFSET
        mem_axi_lite_write(0, 12'h00C, {16'h0010, 4'h0, 4'h0});     // tdest=8'h10, opcode=0 (WRITE_IFB)
        mem_axi_lite_write(0, 12'h010, 32'h0000_0001);              // TRIGGER
        // poll STATUS[1]=cmd_done_sticky
        do begin
            mem_axi_lite_read(0, 12'h014, status);
        end while (!status[1]);
    endtask

    // 主流程
    int errors = 0;
    initial begin
        string case_dir = "../tb_core_dma/cases/case01";
        int    h_in_total = 240;          // case01 H_IN_TOTAL
        int    row_words  = 135;          // case01 W_IN × cin_slices = 135 × 1
        int    mismatch = 0;

        csr_awaddr = 0; csr_awvalid = 0;
        csr_wdata = 0; csr_wstrb = 0; csr_wvalid = 0; csr_bready = 0;
        mem_csr_awaddr = 0; mem_csr_awvalid = 0;
        mem_csr_wdata = 0; mem_csr_wstrb = 0; mem_csr_wvalid = 0; mem_csr_bready = 0;
        #20 rst_n = 1;
        #10;

        $display("============================================================");
        $display("== tb_mesh_single_conv: case01 (mesh path) ==");
        $display("============================================================");

        // 1. 解析 config + preload data
        load_config(case_dir);
        $display("  case meta: ifb=%0d wb=%0d ofb=%0d rdma=%0d",
                 ifb_words_cfg, wb_words_cfg, ofb_words_cfg, rdma_words_cfg);
        preload_ddr(case_dir);
        $display("  preload done");

        // 2. host 配 ConvCore[0]:
        //   - SKIP_IDMA=1 (cross-core consumer, IFB 由 mesh push)
        //   - IFB_STRIP_ROWS / IFB_RING_WORDS (mesh 模式 host 旁路写, 不进 desc)
        //   - OFM_TDEST = (y=0, x=0) = Mem[0]; OFM_OPCODE = WRITE_DDR_OFB (0x5)
        axi_lite_write(0, ADDR_SKIP_IDMA,      32'd1);
        axi_lite_write(0, ADDR_IFB_STRIP_ROWS, 32'd6);
        axi_lite_write(0, ADDR_IFB_RING_WORDS, 32'd810);
        axi_lite_write(0, ADDR_OFM_TDEST,      32'h0000_0000);
        axi_lite_write(0, ADDR_OFM_OPCODE,     32'h0000_0005);

        // 3. host 写 DESC_LIST_BASE + 启动 DFE
        axi_lite_write(0, ADDR_DESC_LIST_BASE, DDR_DESC_BASE);
        axi_lite_write(0, ADDR_CTRL, 32'h0000_0010);   // start_dfe
        do @(posedge clk); while (u_dut.gen_core[0].u_conv.u_core.dfe_busy == 1'b0);
        do @(posedge clk); while (u_dut.gen_core[0].u_conv.u_core.dfe_busy == 1'b1);
        $display("  DFE done");

        // 4. host 启动 layer (CTRL[5]=1)
        axi_lite_write(0, ADDR_CTRL, 32'h0000_0020);
        $display("  layer started");

        // 5. fork: TB 推 IFB rows; main flow 等 layer_done
        fork
            // 推 IFB row 0..H_IN-1, 每 row 一个 packet
            begin
                for (int r = 0; r < h_in_total; r++) begin
                    mem_send_ifb_row(r, row_words, r * row_words);
                end
                $display("  all %0d IFB rows pushed", h_in_total);
            end
            // 等 done
            begin
                wait (done_per_core[0] == 1'b1);
                $display("  ConvCore[0] done @ t=%0t", $time);
            end
        join_any
        disable fork;

        repeat (50) @(posedge clk);

        // 6. bit-exact 比对 mem_core[0].ddr_mem (OFB 区) vs expected_ofm
        $display("\n[Check OFM bit-exact]");
        for (int i = 0; i < ofb_words_cfg; i++) begin
            automatic logic [BUS_DATA_W-1:0] got = u_dut.gen_mem[0].u_mem.ddr_mem[(DDR_OFB_BASE/16) + i];
            automatic logic [OFB_WIDTH-1:0]  exp = exp_arr[i];
            if (got[OFB_WIDTH-1:0] !== exp) begin
                if (mismatch < 5)
                    $display("  FAIL ofb[%0d]=%h expect %h", i, got[OFB_WIDTH-1:0], exp);
                mismatch++;
            end
        end
        if (mismatch == 0)
            $display("  PASS: %0d OFM words bit-exact", ofb_words_cfg);
        else
            errors += mismatch;

        $display("\n============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (single conv mesh end-to-end)");
        else
            $display("  RESULT: FAIL  errors=%0d", errors);
        $display("============================================================");
        $finish;
    end

    initial begin
        #500_000_000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
