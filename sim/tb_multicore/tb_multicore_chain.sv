`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// tb_multicore_chain.sv  --  M2.5 P0 多核多层 chain TB (通用 N 核 N 层)
//
// 跑 toolchain/run_multicore_chain.py 生成的 case 目录:
//   cases/<name>/multicore_meta.txt      (NUM_CORES, NUM_LAYERS, DDR layout, ...)
//   cases/<name>/chain_data/layer<N>/    (每层独立 ifb/wb/expected_ofm/rdma)
//   cases/<name>/core<i>/desc_list.hex   (核 i 的多 layer desc list)
//
// 工作流:
//   1. 解析 multicore_meta.txt
//   2. 把每层 ifb / wb / rdma 数据 preload 到 DDR (按 meta 中各层 base 摆好)
//   3. 把每核 desc list preload 到 DDR (CORE_<i>_DESC_BASE)
//   4. 启所有核 dfe 拉 desc + start_layer (consumer 先启的策略由 driver 保证)
//   5. 等所有核 done
//   6. 比对 FINAL_OFM @ DDR_FINAL_OFM_BASE 跟最后一层 expected_ofm.txt
// =============================================================================

module tb_multicore_chain;
    timeunit 1ns; timeprecision 1ps;

    // ----------------- 参数 (从 svh) -----------------
    localparam int NUM_COL    = `FLUX_NUM_COL;
    localparam int NUM_PE     = `FLUX_NUM_PE;
    localparam int DATA_WIDTH = `FLUX_DATA_WIDTH;
    localparam int PSUM_WIDTH = `FLUX_PSUM_WIDTH;
    localparam int SRAM_DEPTH = `FLUX_IFB_DEPTH;
    localparam int CSR_DATA_W = `FLUX_CSR_DATA_W;
    localparam int BUS_ADDR_W = `FLUX_BUS_ADDR_W;
    localparam int BUS_DATA_W = `FLUX_BUS_DATA_W;
    localparam int AXI_M_ID   = `FLUX_AXI_M_ID;
    localparam int AXI_M_W    = `FLUX_AXI_M_WIDTH;

    localparam int NUM_CORES  = 2;        // P0 demo: 跟 driver 默认一致
    localparam int CORE_ID_W  = 1;
    localparam int HOST_CSR_AW = 12 + CORE_ID_W;
    localparam int CORE_BUS_ID = AXI_M_ID + AXI_M_W;
    localparam int EXT_BUS_ID  = CORE_BUS_ID + CORE_ID_W;
    localparam int DDR_DEPTH   = 1048576*32;        // 512 MB DDR (足够装多核数据)

    // cfg_regs 关键地址 (从 svh)
    localparam [11:0] ADDR_CTRL             = `FLUX_ADDR_CTRL;
    localparam [11:0] ADDR_DESC_LIST_BASE   = `FLUX_ADDR_DESC_LIST_BASE;
    localparam [11:0] ADDR_DESC_COUNT       = `FLUX_ADDR_DESC_COUNT;

    // ----------------- 时钟复位 -----------------
    logic clk = 0;  always #5 clk = ~clk;
    logic rst_n = 0;

    // ----------------- multicore_top 信号 -----------------
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

    logic [EXT_BUS_ID-1:0]    bus_awid, bus_bid, bus_arid, bus_rid;
    logic [BUS_ADDR_W-1:0]    bus_awaddr, bus_araddr;
    logic [7:0]               bus_awlen, bus_arlen;
    logic [1:0]               bus_awburst, bus_arburst, bus_bresp, bus_rresp;
    logic                     bus_awvalid, bus_awready, bus_wvalid, bus_wready, bus_wlast;
    logic                     bus_bvalid, bus_bready, bus_arvalid, bus_arready;
    logic                     bus_rvalid, bus_rready, bus_rlast;
    logic [BUS_DATA_W-1:0]    bus_wdata, bus_rdata;
    logic [BUS_DATA_W/8-1:0]  bus_wstrb;

    logic [NUM_CORES-1:0]     done_per_core;

    // ----------------- DUT -----------------
    multicore_top #(
        .NUM_CORES(NUM_CORES)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb), .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp), .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),
        .bus_awid(bus_awid), .bus_awaddr(bus_awaddr), .bus_awlen(bus_awlen),
        .bus_awburst(bus_awburst), .bus_awvalid(bus_awvalid), .bus_awready(bus_awready),
        .bus_wdata(bus_wdata), .bus_wstrb(bus_wstrb), .bus_wlast(bus_wlast),
        .bus_wvalid(bus_wvalid), .bus_wready(bus_wready),
        .bus_bid(bus_bid), .bus_bresp(bus_bresp), .bus_bvalid(bus_bvalid), .bus_bready(bus_bready),
        .bus_arid(bus_arid), .bus_araddr(bus_araddr), .bus_arlen(bus_arlen),
        .bus_arburst(bus_arburst), .bus_arvalid(bus_arvalid), .bus_arready(bus_arready),
        .bus_rid(bus_rid), .bus_rdata(bus_rdata), .bus_rresp(bus_rresp), .bus_rlast(bus_rlast),
        .bus_rvalid(bus_rvalid), .bus_rready(bus_rready),
        .done_per_core(done_per_core)
    );

    // ----------------- DDR mock -----------------
    axi_slave_mem #(
        .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .ID_W(EXT_BUS_ID), .DEPTH(DDR_DEPTH)
    ) u_ddr (
        .clk(clk), .rstn(rst_n),
        .AWID(bus_awid), .AWADDR(bus_awaddr), .AWLEN(bus_awlen), .AWBURST(bus_awburst),
        .AWVALID(bus_awvalid), .AWREADY(bus_awready),
        .WDATA(bus_wdata), .WSTRB(bus_wstrb), .WLAST(bus_wlast),
        .WVALID(bus_wvalid), .WREADY(bus_wready),
        .BID(bus_bid), .BRESP(bus_bresp), .BVALID(bus_bvalid), .BREADY(bus_bready),
        .ARID(bus_arid), .ARADDR(bus_araddr), .ARLEN(bus_arlen), .ARBURST(bus_arburst),
        .ARVALID(bus_arvalid), .ARREADY(bus_arready),
        .RID(bus_rid), .RDATA(bus_rdata), .RRESP(bus_rresp), .RLAST(bus_rlast),
        .RVALID(bus_rvalid), .RREADY(bus_rready)
    );

    // ----------------- Preload buffers -----------------
    logic [BUS_DATA_W-1:0]   ifb_arr  [0:65535];
    logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_arr [0:4095];
    logic [NUM_COL*DATA_WIDTH-1:0]        exp_arr [0:65535];
    logic [BUS_DATA_W-1:0]   desc_arr [0:8191];
    logic [BUS_DATA_W-1:0]   rdma_arr [0:1023];

    // ----------------- multicore_meta.txt 解析 -----------------
    int n_layers;
    int core_n_layers [0:7];
    int core_desc_count [0:7];
    longint ddr_input_base;
    longint ddr_final_ofm_base;
    longint core_desc_base [0:7];
    int final_h_out, final_w_out, final_c_out;
    int layer_ifb_words [0:31];
    int layer_wb_words  [0:31];
    int layer_ofb_words [0:31];
    longint layer_ddr_ifb [0:31];
    longint layer_ddr_wb  [0:31];
    longint layer_ddr_ofb [0:31];
    longint layer_ddr_rdma[0:31];
    string  layer_dirs    [0:31];

    task automatic parse_meta(input string case_dir);
        int     fd;
        string  line, key, val_s, suffix;
        longint val;
        int     layer_id, core_id, p, p1, i_ch;
        string  path;
        bit     parsed_num;

        path = $sformatf("%s/multicore_meta.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("FATAL: cannot open %s", path); $stop; end
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            parsed_num = 1'b0;
            // 格式 "KEY = VALUE" 或 "KEY = 0xHEX" 或 "KEY = string"
            if ($sscanf(line, "%s = 0x%h", key, val) == 2) parsed_num = 1'b1;
            else if ($sscanf(line, "%s = %d", key, val) == 2) parsed_num = 1'b1;
            else if ($sscanf(line, "%s = %s", key, val_s) == 2) begin
                // string (LAYER_X_DIR)
                if (key.len() > 6 && key.substr(0,5) == "LAYER_") begin
                    p1 = -1;
                    for (int i = 6; i < key.len(); i++)
                        if (key.getc(i) == "_") begin p1 = i; break; end
                    if (p1 > 0) begin
                        layer_id = key.substr(6, p1-1).atoi();
                        if (key.substr(p1+1, key.len()-1) == "DIR")
                            layer_dirs[layer_id] = val_s;
                    end
                end
                continue;
            end else continue;

            if (!parsed_num) continue;

            // 数字键
            case (key)
                "NUM_LAYERS"        : n_layers = val;
                "DDR_INPUT_BASE"    : ddr_input_base = val;
                "DDR_FINAL_OFM_BASE": ddr_final_ofm_base = val;
                "FINAL_H_OUT"       : final_h_out = val;
                "FINAL_W_OUT"       : final_w_out = val;
                "FINAL_C_OUT"       : final_c_out = val;
                default: begin
                    if (key.len() > 5 && key.substr(0, 4) == "CORE_") begin
                        p = -1;
                        for (int i = 5; i < key.len(); i++)
                            if (key.getc(i) == "_") begin p = i; break; end
                        if (p > 0) begin
                            core_id = key.substr(5, p-1).atoi();
                            suffix = key.substr(p+1, key.len()-1);
                            if (suffix == "DESC_BASE") core_desc_base[core_id] = val;
                            else if (suffix == "N_LAYERS") core_n_layers[core_id] = val;
                            else if (suffix == "DESC_COUNT") core_desc_count[core_id] = val;
                        end
                    end else if (key.len() > 6 && key.substr(0, 5) == "LAYER_") begin
                        p = -1;
                        for (int i = 6; i < key.len(); i++)
                            if (key.getc(i) == "_") begin p = i; break; end
                        if (p > 0) begin
                            layer_id = key.substr(6, p-1).atoi();
                            suffix = key.substr(p+1, key.len()-1);
                            case (suffix)
                                "IFB_WORDS"  : layer_ifb_words[layer_id] = val;
                                "WB_WORDS"   : layer_wb_words[layer_id] = val;
                                "OFB_WORDS"  : layer_ofb_words[layer_id] = val;
                                "DDR_IFB"    : layer_ddr_ifb[layer_id] = val;
                                "DDR_WB"     : layer_ddr_wb[layer_id] = val;
                                "DDR_OFB"    : layer_ddr_ofb[layer_id] = val;
                                "DDR_RDMA"   : layer_ddr_rdma[layer_id] = val;
                                default      : ;
                            endcase
                        end
                    end
                end
            endcase
        end
        $fclose(fd);
        $display("[META] n_layers=%0d, n_cores=%0d", n_layers, NUM_CORES);
        for (int c = 0; c < NUM_CORES; c++)
            $display("  Core %0d: %0d layers, desc_base=0x%h", c, core_n_layers[c], core_desc_base[c]);
        for (int l = 0; l < n_layers; l++)
            $display("  Layer %0d: ifb=%0d wb=%0d ofb=%0d  ifb_addr=0x%h wb_addr=0x%h ofb_addr=0x%h",
                     l, layer_ifb_words[l], layer_wb_words[l], layer_ofb_words[l],
                     layer_ddr_ifb[l], layer_ddr_wb[l], layer_ddr_ofb[l]);
    endtask

    // ----------------- DDR preload helpers -----------------
    task automatic preload_ifb(input string layer_dir, input longint base, input int n_words);
        int base_w = base / 16;
        $readmemh($sformatf("%s/ifb.txt", layer_dir), ifb_arr);
        for (int i = 0; i < n_words; i++) u_ddr.mem[base_w + i] = ifb_arr[i];
    endtask

    task automatic preload_wb(input string layer_dir, input longint base, input int n_words);
        int base_w = base / 16;
        $readmemh($sformatf("%s/wb.txt", layer_dir), wb_arr);
        for (int i = 0; i < n_words; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];
    endtask

    task automatic preload_rdma(input string layer_dir, input longint base);
        int base_w = base / 16;
        int fd = $fopen($sformatf("%s/rdma_data.txt", layer_dir), "r");
        if (fd == 0) return;
        $fclose(fd);
        $readmemh($sformatf("%s/rdma_data.txt", layer_dir), rdma_arr);
        // 读取 cout_slices × 4 word; 这里偷懒读 1024
        for (int i = 0; i < 32; i++) u_ddr.mem[base_w + i] = rdma_arr[i];
    endtask

    task automatic preload_desc(input string case_dir, input int core_id, input longint base);
        int base_w = base / 16;
        $readmemh($sformatf("%s/core%0d/desc_list.hex", case_dir, core_id), desc_arr);
        // 用一个大上界, 实际跑不超过 8191 beats
        for (int i = 0; i < 8192; i++) u_ddr.mem[base_w + i] = desc_arr[i];
    endtask

    // ----------------- AXI-Lite write helper -----------------
    task automatic axi_lite_write(input bit core_id, input [11:0] reg_addr, input [31:0] data);
        csr_awaddr  <= {core_id, reg_addr};
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

    // 启核: 写 boot regs + start_dfe + 等 desc 灌完 + start_layer
    // desc_count 用一个保守大上界 (driver 传入实际值, 这里假设没 over)
    task automatic start_core(input bit core_id, input longint desc_base, input int desc_count);
        axi_lite_write(core_id, ADDR_DESC_LIST_BASE, desc_base);
        axi_lite_write(core_id, ADDR_DESC_COUNT, desc_count);
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0010);   // start_dfe
        if (core_id == 1'b0) begin
            wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b1);
            wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b0);
        end else begin
            wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b1);
            wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b0);
        end
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0020);   // start_layer
    endtask

    // ----------------- 比对 final OFM -----------------
    task automatic check_final_ofb(input string case_dir, output int mismatches);
        // 最后一层 expected_ofm.txt 在 chain_data/layer<N-1>/
        string final_dir;
        int ofb_base_w;
        int n_words = layer_ofb_words[n_layers - 1];
        logic [NUM_COL*DATA_WIDTH-1:0] expected, got;
        final_dir = layer_dirs[n_layers - 1];
        if (final_dir == "") final_dir = $sformatf("%s/chain_data/layer%02d", case_dir, n_layers - 1);
        ofb_base_w = ddr_final_ofm_base / 16;
        $readmemh($sformatf("%s/expected_ofm.txt", final_dir), exp_arr);
        mismatches = 0;
        for (int i = 0; i < n_words; i++) begin
            expected = exp_arr[i];
            got      = u_ddr.mem[ofb_base_w + i][NUM_COL*DATA_WIDTH-1:0];
            if (got !== expected) begin
                if (mismatches < 5)
                    $display("  FINAL.OFB[%0d]: expect=%h got=%h", i, expected, got);
                mismatches++;
            end
        end
    endtask

    // ----------------- 主流程 -----------------
    initial begin
        string case_dir;
        string ldir;
        int    mismatches;
        longint t_start;

        case_dir = "cases/multicore_demo";

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_multicore_chain: multi-layer chain N=%0d ==", NUM_CORES);

        parse_meta(case_dir);

        // 各层数据 preload
        for (int l = 0; l < n_layers; l++) begin
            ldir = layer_dirs[l];
            if (ldir == "") ldir = $sformatf("%s/chain_data/layer%02d", case_dir, l);
            // 第一层 IFB 必须 preload (整网入口); 中间层 IFB 由上层 OFM 写入 DDR 形成
            if (l == 0)
                preload_ifb(ldir, layer_ddr_ifb[l], layer_ifb_words[l]);
            preload_wb(ldir, layer_ddr_wb[l], layer_wb_words[l]);
            preload_rdma(ldir, layer_ddr_rdma[l]);
        end

        // 各核 desc preload
        for (int c = 0; c < NUM_CORES; c++)
            preload_desc(case_dir, c, core_desc_base[c]);

        // 清 final OFM 区
        for (int i = 0; i < layer_ofb_words[n_layers - 1]; i++)
            u_ddr.mem[ddr_final_ofm_base/16 + i] = '0;

        $display("  DDR preload done. Starting all cores (consumer-first by core_id desc)...");

        t_start = $time;

        // 启动策略: 所有核**同时**启动 (race start), 然后 wait 全部 done.
        // mode C 切片下各核独立从 DDR 读自己段, 互不依赖, 同时跑.
        // (旧 P0 sequential 启动已废弃, 没有并发收益)
        for (int c = 0; c < NUM_CORES; c++) begin
            start_core(c[0], core_desc_base[c], core_desc_count[c]);
            $display("  Core %0d started @ t=%0t (desc_count=%0d)", c, $time, core_desc_count[c]);
        end
        wait (&done_per_core);
        $display("  ALL cores done @ t=%0t", $time);

        // 等所有核 done
        wait (&done_per_core);
        $display("  ALL cores done @ t=%0t (cycles=%0d)", $time, ($time - t_start) / 10);

        @(posedge clk); @(posedge clk);

        // DEBUG: dump core 1 IFB SRAM 头几个 word vs L1 expected ifb (= L0 expected ofm)
        begin
            string ldir;
            ldir = layer_dirs[0];
            if (ldir == "") ldir = $sformatf("%s/chain_data/layer00", case_dir);
            $readmemh($sformatf("%s/expected_ofm.txt", ldir), exp_arr);
            for (int i = 0; i < 3; i++) begin
                $display("  C1.IFB.SRAM[%0d] = %h", i, u_dut.gen_core[1].u_core.u_ifb.mem[i]);
                $display("  L0.expected_ofm[%0d] = %h", i, exp_arr[i]);
            end
        end

        // DEBUG: 比对每层中间 OFM
        begin
            string ldir;
            int    ofb_base_w;
            int    n_w;
            int    mm;
            logic [NUM_COL*DATA_WIDTH-1:0] expected, got;
            for (int l = 0; l < n_layers - 1; l++) begin
                ofb_base_w = layer_ddr_ofb[l] / 16;
                n_w = layer_ofb_words[l];
                mm = 0;
                ldir = layer_dirs[l];
                if (ldir == "") ldir = $sformatf("%s/chain_data/layer%02d", case_dir, l);
                $readmemh($sformatf("%s/expected_ofm.txt", ldir), exp_arr);
                for (int i = 0; i < n_w; i++) begin
                    expected = exp_arr[i];
                    got      = u_ddr.mem[ofb_base_w + i][NUM_COL*DATA_WIDTH-1:0];
                    if (got !== expected) mm++;
                end
                $display("  Layer %0d intermediate OFM @ 0x%h: mismatches=%0d/%0d  first got=%h vs exp=%h",
                         l, layer_ddr_ofb[l], mm, n_w,
                         u_ddr.mem[ofb_base_w + 0][NUM_COL*DATA_WIDTH-1:0],
                         exp_arr[0]);
            end
        end

        check_final_ofb(case_dir, mismatches);

        $display("");
        $display("============================================================");
        if (mismatches == 0)
            $display("  RESULT: PASS  (Final OFB matches golden, %0d words checked)",
                     layer_ofb_words[n_layers - 1]);
        else
            $display("  RESULT: FAIL  Final OFB mismatches=%0d / %0d",
                     mismatches, layer_ofb_words[n_layers - 1]);
        $display("  Wall time: %0d ns (%0d cycles @ 10 ns)",
                 $time - t_start, ($time - t_start)/10);
        $display("============================================================");

        $finish;
    end

    // 周期 dump 关键信号
    initial begin
        @(posedge rst_n);
        forever begin
            #500_000;
            $display("[t=%0t] C0 seq.st=%0d cfg_skip_idma=%b done=%b lb_done=%b ow_done=%b idma_done=%b odma_done=%b | C1 seq.st=%0d cfg_skip_idma=%b done=%b lb_done=%b ow_done=%b idma_done=%b odma_done=%b",
                     $time,
                     u_dut.gen_core[0].u_core.u_sequencer.state,
                     u_dut.gen_core[0].u_core.cfg_skip_idma,
                     done_per_core[0],
                     u_dut.gen_core[0].u_core.lb_done,
                     u_dut.gen_core[0].u_core.ow_done,
                     u_dut.gen_core[0].u_core.idma_done,
                     u_dut.gen_core[0].u_core.odma_done,
                     u_dut.gen_core[1].u_core.u_sequencer.state,
                     u_dut.gen_core[1].u_core.cfg_skip_idma,
                     done_per_core[1],
                     u_dut.gen_core[1].u_core.lb_done,
                     u_dut.gen_core[1].u_core.ow_done,
                     u_dut.gen_core[1].u_core.idma_done,
                     u_dut.gen_core[1].u_core.odma_done);
            $display("[t=%0t]   C0 ifb_slv.AW[v/r=%b/%b] rows_pushed=%0d rows_cons=%0d | C1 ifb_slv.AW[v/r=%b/%b] rows_pushed=%0d rows_cons=%0d",
                     $time,
                     u_dut.gen_core[0].u_core.u_ifb_axi_slv.AWVALID,
                     u_dut.gen_core[0].u_core.u_ifb_axi_slv.AWREADY,
                     u_dut.gen_core[0].u_core.u_ifb_axi_slv.rows_pushed,
                     u_dut.gen_core[0].u_core.rows_consumed,
                     u_dut.gen_core[1].u_core.u_ifb_axi_slv.AWVALID,
                     u_dut.gen_core[1].u_core.u_ifb_axi_slv.AWREADY,
                     u_dut.gen_core[1].u_core.u_ifb_axi_slv.rows_pushed,
                     u_dut.gen_core[1].u_core.rows_consumed);
        end
    end

    // 总 watchdog
    initial begin
        #5_000_000;
        $display("FATAL: watchdog timeout @ %0t", $time);
        $stop;
    end

endmodule
