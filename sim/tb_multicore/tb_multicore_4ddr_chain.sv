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

    localparam int NUM_CORES  = 4;        // PoC 多 DDR: 固定 N=4
    localparam int NUM_DDR    = 4;
    localparam int CORE_ID_W  = (NUM_CORES <= 1) ? 1 : $clog2(NUM_CORES);
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

    // 4 个独立 DDR master 信号 (packed array, slot[i] = bits[i*W +: W])
    logic [NUM_DDR*EXT_BUS_ID-1:0]    bus_ddr_awid, bus_ddr_bid, bus_ddr_arid, bus_ddr_rid;
    logic [NUM_DDR*BUS_ADDR_W-1:0]    bus_ddr_awaddr, bus_ddr_araddr;
    logic [NUM_DDR*8-1:0]             bus_ddr_awlen, bus_ddr_arlen;
    logic [NUM_DDR*2-1:0]             bus_ddr_awburst, bus_ddr_arburst, bus_ddr_bresp, bus_ddr_rresp;
    logic [NUM_DDR-1:0]               bus_ddr_awvalid, bus_ddr_awready;
    logic [NUM_DDR-1:0]               bus_ddr_wvalid, bus_ddr_wready, bus_ddr_wlast;
    logic [NUM_DDR-1:0]               bus_ddr_bvalid, bus_ddr_bready;
    logic [NUM_DDR-1:0]               bus_ddr_arvalid, bus_ddr_arready;
    logic [NUM_DDR-1:0]               bus_ddr_rvalid, bus_ddr_rready, bus_ddr_rlast;
    logic [NUM_DDR*BUS_DATA_W-1:0]    bus_ddr_wdata, bus_ddr_rdata;
    logic [NUM_DDR*(BUS_DATA_W/8)-1:0]bus_ddr_wstrb;

    logic [NUM_CORES-1:0]             done_per_core;

    // ----------------- DUT (4-DDR PoC variant) -----------------
    multicore_top_4ddr u_dut (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb), .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp), .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),
        .bus_ddr_awid(bus_ddr_awid), .bus_ddr_awaddr(bus_ddr_awaddr), .bus_ddr_awlen(bus_ddr_awlen),
        .bus_ddr_awburst(bus_ddr_awburst), .bus_ddr_awvalid(bus_ddr_awvalid), .bus_ddr_awready(bus_ddr_awready),
        .bus_ddr_wdata(bus_ddr_wdata), .bus_ddr_wstrb(bus_ddr_wstrb), .bus_ddr_wlast(bus_ddr_wlast),
        .bus_ddr_wvalid(bus_ddr_wvalid), .bus_ddr_wready(bus_ddr_wready),
        .bus_ddr_bid(bus_ddr_bid), .bus_ddr_bresp(bus_ddr_bresp),
        .bus_ddr_bvalid(bus_ddr_bvalid), .bus_ddr_bready(bus_ddr_bready),
        .bus_ddr_arid(bus_ddr_arid), .bus_ddr_araddr(bus_ddr_araddr), .bus_ddr_arlen(bus_ddr_arlen),
        .bus_ddr_arburst(bus_ddr_arburst), .bus_ddr_arvalid(bus_ddr_arvalid), .bus_ddr_arready(bus_ddr_arready),
        .bus_ddr_rid(bus_ddr_rid), .bus_ddr_rdata(bus_ddr_rdata), .bus_ddr_rresp(bus_ddr_rresp),
        .bus_ddr_rlast(bus_ddr_rlast), .bus_ddr_rvalid(bus_ddr_rvalid), .bus_ddr_rready(bus_ddr_rready),
        .done_per_core(done_per_core)
    );

    // ----------------- 4 个独立 DDR mock (sim 测多端口带宽收益的关键!) -----------------
    // 每个 mem 256 MB (DDR_DEPTH/4 words = 8M words × 16 byte). 各自 wptr/rptr/state 独立,
    // 4 个 mem 真并行处理 → 4 倍带宽上限.
    genvar gd;
    generate
        for (gd = 0; gd < NUM_DDR; gd++) begin : g_ddr
            axi_slave_mem #(
                .ADDR_W(BUS_ADDR_W), .DATA_W(BUS_DATA_W), .ID_W(EXT_BUS_ID),
                .DEPTH(DDR_DEPTH/NUM_DDR)
            ) u_mem (
                .clk(clk), .rstn(rst_n),
                .AWID   (bus_ddr_awid   [gd*EXT_BUS_ID +: EXT_BUS_ID]),
                .AWADDR (bus_ddr_awaddr [gd*BUS_ADDR_W +: BUS_ADDR_W]),
                .AWLEN  (bus_ddr_awlen  [gd*8         +: 8]),
                .AWBURST(bus_ddr_awburst[gd*2         +: 2]),
                .AWVALID(bus_ddr_awvalid[gd]), .AWREADY(bus_ddr_awready[gd]),
                .WDATA  (bus_ddr_wdata  [gd*BUS_DATA_W +: BUS_DATA_W]),
                .WSTRB  (bus_ddr_wstrb  [gd*(BUS_DATA_W/8) +: (BUS_DATA_W/8)]),
                .WLAST  (bus_ddr_wlast  [gd]),
                .WVALID (bus_ddr_wvalid [gd]), .WREADY (bus_ddr_wready[gd]),
                .BID    (bus_ddr_bid    [gd*EXT_BUS_ID +: EXT_BUS_ID]),
                .BRESP  (bus_ddr_bresp  [gd*2         +: 2]),
                .BVALID (bus_ddr_bvalid [gd]), .BREADY (bus_ddr_bready[gd]),
                .ARID   (bus_ddr_arid   [gd*EXT_BUS_ID +: EXT_BUS_ID]),
                .ARADDR (bus_ddr_araddr [gd*BUS_ADDR_W +: BUS_ADDR_W]),
                .ARLEN  (bus_ddr_arlen  [gd*8         +: 8]),
                .ARBURST(bus_ddr_arburst[gd*2         +: 2]),
                .ARVALID(bus_ddr_arvalid[gd]), .ARREADY(bus_ddr_arready[gd]),
                .RID    (bus_ddr_rid    [gd*EXT_BUS_ID +: EXT_BUS_ID]),
                .RDATA  (bus_ddr_rdata  [gd*BUS_DATA_W +: BUS_DATA_W]),
                .RRESP  (bus_ddr_rresp  [gd*2         +: 2]),
                .RLAST  (bus_ddr_rlast  [gd]),
                .RVALID (bus_ddr_rvalid [gd]), .RREADY (bus_ddr_rready[gd])
            );
        end
    endgenerate

    // ----------------- Preload buffers -----------------
    logic [BUS_DATA_W-1:0]   ifb_arr  [0:524287];   // ResNet Patch IFB 960×540×1 = 518400 words
    logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_arr [0:4095];
    logic [NUM_COL*DATA_WIDTH-1:0]        exp_arr [0:65535];
    logic [BUS_DATA_W-1:0]   desc_arr [0:8191];
    logic [BUS_DATA_W-1:0]   rdma_arr [0:65535];   // residual: bias + H_OUT*W_OUT*cout_slices

    // ----------------- multicore_meta.txt 解析 -----------------
    int n_layers;
    int multi_ddr_mode;          // 0 = 单 DDR (slot 0 only), 1 = 4 DDR broadcast
    longint ddr_input_base;
    longint ddr_final_ofm_base;
    int final_h_out, final_w_out, final_c_out;
    int layer_ifb_words [0:31];
    int layer_wb_words  [0:31];
    int layer_ofb_words [0:31];
    longint layer_ddr_ifb [0:31];
    longint layer_ddr_wb  [0:31];
    longint layer_ddr_ofb [0:31];
    longint layer_ddr_rdma[0:31];
    int     layer_rdma_words [0:31];   // bias + opt shortcut (residual layer 时变大)
    int     layer_preload_ifb[0:31];   // 1 表示该 layer IFB 是 root (random) 需 host preload
    string  layer_dirs    [0:31];

    // Per-(core, layer) desc list base + count. count==0 表示该 core 该层不参与.
    longint core_layer_desc_base  [0:7][0:31];
    int     core_layer_desc_count [0:7][0:31];
    // Per-(core, layer) sliced RDMA (residual + W slice 时每核独立 rdma_data 文件)
    longint core_layer_rdma_base  [0:7][0:31];
    int     core_layer_rdma_words [0:7][0:31];

    task automatic parse_meta(input string case_dir);
        int     fd;
        string  line, key, val_s, suffix;
        longint val;
        int     layer_id, core_id, p, p1, p2, i_ch;
        string  path;
        bit     parsed_num;

        path = $sformatf("%s/multicore_meta.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("FATAL: cannot open %s", path); $stop; end
        // 初始化 desc count 为 0
        for (int c = 0; c < 8; c++)
            for (int l = 0; l < 32; l++) begin
                core_layer_desc_base[c][l]  = 0;
                core_layer_desc_count[c][l] = 0;
                core_layer_rdma_base[c][l]  = 0;
                core_layer_rdma_words[c][l] = 0;
            end
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
                "MULTI_DDR"         : multi_ddr_mode = val;
                "DDR_INPUT_BASE"    : ddr_input_base = val;
                "DDR_FINAL_OFM_BASE": ddr_final_ofm_base = val;
                "FINAL_H_OUT"       : final_h_out = val;
                "FINAL_W_OUT"       : final_w_out = val;
                "FINAL_C_OUT"       : final_c_out = val;
                default: begin
                    // CORE_<c>_LAYER_<l>_DESC_BASE / DESC_COUNT
                    if (key.len() > 11 && key.substr(0, 4) == "CORE_") begin
                        // 找 _LAYER_
                        p1 = -1; p2 = -1;
                        for (int i = 5; i < key.len(); i++)
                            if (key.getc(i) == "_") begin p1 = i; break; end
                        if (p1 > 0 && p1 + 6 < key.len() &&
                            key.substr(p1+1, p1+5) == "LAYER" && key.getc(p1+6) == "_") begin
                            // CORE_<c>_LAYER_<l>_<suffix>
                            for (int j = p1+7; j < key.len(); j++)
                                if (key.getc(j) == "_") begin p2 = j; break; end
                            if (p2 > 0) begin
                                core_id  = key.substr(5,    p1-1).atoi();
                                layer_id = key.substr(p1+7, p2-1).atoi();
                                suffix   = key.substr(p2+1, key.len()-1);
                                if      (suffix == "DESC_BASE")  core_layer_desc_base [core_id][layer_id] = val;
                                else if (suffix == "DESC_COUNT") core_layer_desc_count[core_id][layer_id] = val;
                                else if (suffix == "RDMA_BASE")  core_layer_rdma_base [core_id][layer_id] = val;
                                else if (suffix == "RDMA_WORDS") core_layer_rdma_words[core_id][layer_id] = val;
                            end
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
                                "RDMA_WORDS" : layer_rdma_words[layer_id] = val;
                                "PRELOAD_IFB": layer_preload_ifb[layer_id] = val;
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
        for (int c = 0; c < NUM_CORES; c++) begin
            for (int l = 0; l < n_layers; l++)
                if (core_layer_desc_count[c][l] > 0)
                    $display("  Core %0d Layer %0d: desc_base=0x%h count=%0d",
                             c, l, core_layer_desc_base[c][l], core_layer_desc_count[c][l]);
        end
        for (int l = 0; l < n_layers; l++)
            $display("  Layer %0d: ifb=%0d wb=%0d ofb=%0d  ifb_addr=0x%h wb_addr=0x%h ofb_addr=0x%h",
                     l, layer_ifb_words[l], layer_wb_words[l], layer_ofb_words[l],
                     layer_ddr_ifb[l], layer_ddr_wb[l], layer_ddr_ofb[l]);
    endtask

    // ----------------- DDR slot 路由 -----------------
    // 4 个独立 axi_slave_mem (g_ddr[0..3].u_mem). addr bits[31:28] 选 slot.
    // 写 helper: 单 slot 写 (按 addr 高位选), 跟 broadcast 写 (4 mem 同位置同数据).
    // SystemVerilog 不支持 dynamic hier ref, 用 case 静态展开.
    task automatic ddr_write_slot(input int slot_idx, input int word_offset,
                                   input logic [BUS_DATA_W-1:0] data);
        case (slot_idx)
            0: g_ddr[0].u_mem.mem[word_offset] = data;
            1: g_ddr[1].u_mem.mem[word_offset] = data;
            2: g_ddr[2].u_mem.mem[word_offset] = data;
            3: g_ddr[3].u_mem.mem[word_offset] = data;
        endcase
    endtask

    function automatic logic [BUS_DATA_W-1:0] ddr_read_slot(input int slot_idx, input int word_offset);
        case (slot_idx)
            0: return g_ddr[0].u_mem.mem[word_offset];
            1: return g_ddr[1].u_mem.mem[word_offset];
            2: return g_ddr[2].u_mem.mem[word_offset];
            3: return g_ddr[3].u_mem.mem[word_offset];
            default: return '0;
        endcase
    endfunction

    // base 完整 32-bit 字节地址 → (slot, intra-slot word offset).
    //   slot     = base[31:28]   (multi-DDR 模式由 driver OR 入)
    //   word_off = base[27:4] / 1   (28-bit slot 内 byte 地址 / 16 byte/word)
    task automatic ddr_write_addr(input longint base, input int word_idx,
                                   input logic [BUS_DATA_W-1:0] data);
        int slot   = (base >> 28) & 'hF;
        int wbase  = (base & 32'h0FFFFFFF) / 16;
        ddr_write_slot(slot, wbase + word_idx, data);
    endtask

    function automatic logic [BUS_DATA_W-1:0] ddr_read_addr(input longint base, input int word_idx);
        int slot   = (base >> 28) & 'hF;
        int wbase  = (base & 32'h0FFFFFFF) / 16;
        return ddr_read_slot(slot, wbase + word_idx);
    endfunction

    // ----------------- DDR preload helpers -----------------
    // multi_ddr_mode=1 时: shared 数据 (input/wb/rdma) broadcast 到 4 个 slot 同位置.
    // multi_ddr_mode=0 时: 按 addr 单 slot 写 (实际都 slot 0, 因为 addr 高位都 0).
    task automatic preload_ifb(input string layer_dir, input longint base, input int n_words);
        $readmemh($sformatf("%s/ifb.txt", layer_dir), ifb_arr);
        if (multi_ddr_mode) begin
            // broadcast: same intra-slot offset on all 4 slots
            automatic int wbase = (base & 32'h0FFFFFFF) / 16;
            for (int s = 0; s < 4; s++)
                for (int i = 0; i < n_words; i++) ddr_write_slot(s, wbase + i, ifb_arr[i]);
        end else begin
            for (int i = 0; i < n_words; i++) ddr_write_addr(base, i, ifb_arr[i]);
        end
    endtask

    task automatic preload_wb(input string layer_dir, input longint base, input int n_words);
        $readmemh($sformatf("%s/wb.txt", layer_dir), wb_arr);
        if (multi_ddr_mode) begin
            automatic int wbase = (base & 32'h0FFFFFFF) / 16;
            for (int s = 0; s < 4; s++)
                for (int i = 0; i < n_words; i++)
                    for (int b = 0; b < 16; b++)
                        ddr_write_slot(s, wbase + i*16 + b, wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W]);
        end else begin
            for (int i = 0; i < n_words; i++)
                for (int b = 0; b < 16; b++)
                    ddr_write_addr(base, i*16 + b, wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W]);
        end
    endtask

    task automatic preload_rdma(input string layer_dir, input longint base, input int n_words);
        automatic int fd = $fopen($sformatf("%s/rdma_data.txt", layer_dir), "r");
        if (fd == 0) return;
        $fclose(fd);
        for (int i = 0; i < 65536; i++) rdma_arr[i] = '0;
        $readmemh($sformatf("%s/rdma_data.txt", layer_dir), rdma_arr);
        if (multi_ddr_mode) begin
            automatic int wbase = (base & 32'h0FFFFFFF) / 16;
            for (int s = 0; s < 4; s++)
                for (int i = 0; i < n_words; i++) ddr_write_slot(s, wbase + i, rdma_arr[i]);
        end else begin
            for (int i = 0; i < n_words; i++) ddr_write_addr(base, i, rdma_arr[i]);
        end
    endtask

    // Per-(core, layer) sliced RDMA: residual + W slice 时每核独立 rdma_data_c<core>.txt
    // 这是 per-core 数据, base 已含 core slot 偏移, 直接写到对应 slot (不 broadcast).
    task automatic preload_core_layer_rdma(input string layer_dir, input int core_id,
                                            input longint base, input int n_words);
        string path;
        path = $sformatf("%s/rdma_data_c%0d.txt", layer_dir, core_id);
        for (int i = 0; i < 65536; i++) rdma_arr[i] = '0;
        $readmemh(path, rdma_arr);
        for (int i = 0; i < n_words; i++) ddr_write_addr(base, i, rdma_arr[i]);
    endtask

    // Per-layer per-core desc list preload. base 已含 core slot 偏移, 单 slot 写.
    task automatic preload_layer_desc(input string case_dir, input int core_id,
                                       input int layer_idx, input longint base);
        string path, l2;
        if (layer_idx < 10) l2 = $sformatf("0%0d", layer_idx);
        else                l2 = $sformatf("%0d",  layer_idx);
        path = $sformatf("%s/core%0d/layer%s_desc_list.hex", case_dir, core_id, l2);
        $readmemh(path, desc_arr);
        for (int i = 0; i < 1024; i++) ddr_write_addr(base, i, desc_arr[i]);
    endtask

    // ----------------- AXI-Lite write helper -----------------
    task automatic axi_lite_write(input [CORE_ID_W-1:0] core_id,
                                   input [11:0] reg_addr,
                                   input [31:0] data);
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

    // ----------------- per-layer dfe + start_layer 同步 -----------------
    // 每一 stage:
    //   1. host 给每个参与的 core 写 DESC_LIST_BASE / DESC_COUNT
    //   2. host 触发所有 core 的 start_dfe (并行触发, 各核独立拉 desc)
    //   3. host 等所有 core dfe_done
    //   4. host 触发所有 core 的 start_layer (同时触发 → 并行计算)
    //   5. host 等 done_per_core 全 1 (sticky 已被 start_layer 清掉, 现在重新置 1 表示 layer_done)
    task automatic axi_lite_write_dfe_start(input [CORE_ID_W-1:0] core_id);
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0010);
    endtask

    task automatic axi_lite_write_layer_start(input [CORE_ID_W-1:0] core_id);
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0020);
    endtask

    // 等指定 core 的 DFE 0→1→0. SystemVerilog 不支持 dynamic hierarchical refs,
    // 只能 case 静态展开 (NUM_CORES 上限 4).
    task automatic wait_dfe_done(input [CORE_ID_W-1:0] core_id);
        case (core_id)
            'd0: begin
                wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b0);
            end
            'd1: begin
                wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b0);
            end
            'd2: if (NUM_CORES > 2) begin
                wait (u_dut.gen_core[2].u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[2].u_core.dfe_busy == 1'b0);
            end
            'd3: if (NUM_CORES > 3) begin
                wait (u_dut.gen_core[3].u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[3].u_core.dfe_busy == 1'b0);
            end
            default: ;
        endcase
    endtask

    // ----------------- profiling 计数器快照 (per-core / per-DDR) -----------------
    // 仿照 tb_core_dma 的 CASE_RESULT/CASE_PROFILE, 但 multicore 维度更多:
    //   LAYER_PROFILE  l=N cores=mask cycles=X  per-core MAC fire / stall  + per-DDR busy
    //   CORE_RESULT    c=N PASS/FAIL cycles=X mac_fire=X mac_util=X arf_w=X parf_f=X ...
    //   DDR_PROFILE    d=N aw_fire=X w_beats=X ar_fire=X r_beats=X busy_cyc=X busy_pct=X
    //
    // genvar hier ref 不能 dynamic, 所以用 case 静态展开 (NUM_CORES/NUM_DDR 上限 4).
    typedef struct {
        int mac_fire;
        int act_fire, act_stall, act_idle;
        int wgt_fire, wgt_stall, wgt_idle;
        int psum_fire, psum_stall, psum_idle;
        int acc_fire, acc_stall, acc_idle;
        int arf_w, arf_r;
        int parf_f, parf_d;
    } core_snap_t;

    typedef struct {
        int aw_fire, w_beats, ar_fire, r_beats, busy_cyc;
    } ddr_snap_t;

    task automatic snap_core(input int c, output core_snap_t s);
        case (c)
            0: begin
                s.mac_fire   = u_dut.gen_core[0].u_core.u_mac_array.mac_fire_cnt;
                s.act_fire   = u_dut.gen_core[0].u_core.u_mac_array.hs_act_fire;
                s.act_stall  = u_dut.gen_core[0].u_core.u_mac_array.hs_act_stall;
                s.act_idle   = u_dut.gen_core[0].u_core.u_mac_array.hs_act_idle;
                s.wgt_fire   = u_dut.gen_core[0].u_core.u_mac_array.hs_wgt_fire;
                s.wgt_stall  = u_dut.gen_core[0].u_core.u_mac_array.hs_wgt_stall;
                s.wgt_idle   = u_dut.gen_core[0].u_core.u_mac_array.hs_wgt_idle;
                s.psum_fire  = u_dut.gen_core[0].u_core.u_mac_array.hs_psum_fire;
                s.psum_stall = u_dut.gen_core[0].u_core.u_mac_array.hs_psum_stall;
                s.psum_idle  = u_dut.gen_core[0].u_core.u_mac_array.hs_psum_idle;
                s.acc_fire   = u_dut.gen_core[0].u_core.u_ofb_writer.hs_acc_fire;
                s.acc_stall  = u_dut.gen_core[0].u_core.u_ofb_writer.hs_acc_stall;
                s.acc_idle   = u_dut.gen_core[0].u_core.u_ofb_writer.hs_acc_idle;
                s.arf_w      = u_dut.gen_core[0].u_core.u_line_buffer.arf_write_cnt;
                s.arf_r      = u_dut.gen_core[0].u_core.u_line_buffer.arf_read_cnt;
                s.parf_f     = u_dut.gen_core[0].u_core.u_parf_accum.fill_fire_cnt;
                s.parf_d     = u_dut.gen_core[0].u_core.u_parf_accum.drain_fire_cnt;
            end
            1: begin
                s.mac_fire   = u_dut.gen_core[1].u_core.u_mac_array.mac_fire_cnt;
                s.act_fire   = u_dut.gen_core[1].u_core.u_mac_array.hs_act_fire;
                s.act_stall  = u_dut.gen_core[1].u_core.u_mac_array.hs_act_stall;
                s.act_idle   = u_dut.gen_core[1].u_core.u_mac_array.hs_act_idle;
                s.wgt_fire   = u_dut.gen_core[1].u_core.u_mac_array.hs_wgt_fire;
                s.wgt_stall  = u_dut.gen_core[1].u_core.u_mac_array.hs_wgt_stall;
                s.wgt_idle   = u_dut.gen_core[1].u_core.u_mac_array.hs_wgt_idle;
                s.psum_fire  = u_dut.gen_core[1].u_core.u_mac_array.hs_psum_fire;
                s.psum_stall = u_dut.gen_core[1].u_core.u_mac_array.hs_psum_stall;
                s.psum_idle  = u_dut.gen_core[1].u_core.u_mac_array.hs_psum_idle;
                s.acc_fire   = u_dut.gen_core[1].u_core.u_ofb_writer.hs_acc_fire;
                s.acc_stall  = u_dut.gen_core[1].u_core.u_ofb_writer.hs_acc_stall;
                s.acc_idle   = u_dut.gen_core[1].u_core.u_ofb_writer.hs_acc_idle;
                s.arf_w      = u_dut.gen_core[1].u_core.u_line_buffer.arf_write_cnt;
                s.arf_r      = u_dut.gen_core[1].u_core.u_line_buffer.arf_read_cnt;
                s.parf_f     = u_dut.gen_core[1].u_core.u_parf_accum.fill_fire_cnt;
                s.parf_d     = u_dut.gen_core[1].u_core.u_parf_accum.drain_fire_cnt;
            end
            2: begin
                s.mac_fire   = u_dut.gen_core[2].u_core.u_mac_array.mac_fire_cnt;
                s.act_fire   = u_dut.gen_core[2].u_core.u_mac_array.hs_act_fire;
                s.act_stall  = u_dut.gen_core[2].u_core.u_mac_array.hs_act_stall;
                s.act_idle   = u_dut.gen_core[2].u_core.u_mac_array.hs_act_idle;
                s.wgt_fire   = u_dut.gen_core[2].u_core.u_mac_array.hs_wgt_fire;
                s.wgt_stall  = u_dut.gen_core[2].u_core.u_mac_array.hs_wgt_stall;
                s.wgt_idle   = u_dut.gen_core[2].u_core.u_mac_array.hs_wgt_idle;
                s.psum_fire  = u_dut.gen_core[2].u_core.u_mac_array.hs_psum_fire;
                s.psum_stall = u_dut.gen_core[2].u_core.u_mac_array.hs_psum_stall;
                s.psum_idle  = u_dut.gen_core[2].u_core.u_mac_array.hs_psum_idle;
                s.acc_fire   = u_dut.gen_core[2].u_core.u_ofb_writer.hs_acc_fire;
                s.acc_stall  = u_dut.gen_core[2].u_core.u_ofb_writer.hs_acc_stall;
                s.acc_idle   = u_dut.gen_core[2].u_core.u_ofb_writer.hs_acc_idle;
                s.arf_w      = u_dut.gen_core[2].u_core.u_line_buffer.arf_write_cnt;
                s.arf_r      = u_dut.gen_core[2].u_core.u_line_buffer.arf_read_cnt;
                s.parf_f     = u_dut.gen_core[2].u_core.u_parf_accum.fill_fire_cnt;
                s.parf_d     = u_dut.gen_core[2].u_core.u_parf_accum.drain_fire_cnt;
            end
            3: begin
                s.mac_fire   = u_dut.gen_core[3].u_core.u_mac_array.mac_fire_cnt;
                s.act_fire   = u_dut.gen_core[3].u_core.u_mac_array.hs_act_fire;
                s.act_stall  = u_dut.gen_core[3].u_core.u_mac_array.hs_act_stall;
                s.act_idle   = u_dut.gen_core[3].u_core.u_mac_array.hs_act_idle;
                s.wgt_fire   = u_dut.gen_core[3].u_core.u_mac_array.hs_wgt_fire;
                s.wgt_stall  = u_dut.gen_core[3].u_core.u_mac_array.hs_wgt_stall;
                s.wgt_idle   = u_dut.gen_core[3].u_core.u_mac_array.hs_wgt_idle;
                s.psum_fire  = u_dut.gen_core[3].u_core.u_mac_array.hs_psum_fire;
                s.psum_stall = u_dut.gen_core[3].u_core.u_mac_array.hs_psum_stall;
                s.psum_idle  = u_dut.gen_core[3].u_core.u_mac_array.hs_psum_idle;
                s.acc_fire   = u_dut.gen_core[3].u_core.u_ofb_writer.hs_acc_fire;
                s.acc_stall  = u_dut.gen_core[3].u_core.u_ofb_writer.hs_acc_stall;
                s.acc_idle   = u_dut.gen_core[3].u_core.u_ofb_writer.hs_acc_idle;
                s.arf_w      = u_dut.gen_core[3].u_core.u_line_buffer.arf_write_cnt;
                s.arf_r      = u_dut.gen_core[3].u_core.u_line_buffer.arf_read_cnt;
                s.parf_f     = u_dut.gen_core[3].u_core.u_parf_accum.fill_fire_cnt;
                s.parf_d     = u_dut.gen_core[3].u_core.u_parf_accum.drain_fire_cnt;
            end
            default: s = '{default:0};
        endcase
    endtask

    task automatic snap_ddr(input int d, output ddr_snap_t s);
        case (d)
            0: begin
                s.aw_fire = g_ddr[0].u_mem.aw_fire; s.w_beats = g_ddr[0].u_mem.w_beats;
                s.ar_fire = g_ddr[0].u_mem.ar_fire; s.r_beats = g_ddr[0].u_mem.r_beats;
                s.busy_cyc = g_ddr[0].u_mem.busy_cyc;
            end
            1: begin
                s.aw_fire = g_ddr[1].u_mem.aw_fire; s.w_beats = g_ddr[1].u_mem.w_beats;
                s.ar_fire = g_ddr[1].u_mem.ar_fire; s.r_beats = g_ddr[1].u_mem.r_beats;
                s.busy_cyc = g_ddr[1].u_mem.busy_cyc;
            end
            2: begin
                s.aw_fire = g_ddr[2].u_mem.aw_fire; s.w_beats = g_ddr[2].u_mem.w_beats;
                s.ar_fire = g_ddr[2].u_mem.ar_fire; s.r_beats = g_ddr[2].u_mem.r_beats;
                s.busy_cyc = g_ddr[2].u_mem.busy_cyc;
            end
            3: begin
                s.aw_fire = g_ddr[3].u_mem.aw_fire; s.w_beats = g_ddr[3].u_mem.w_beats;
                s.ar_fire = g_ddr[3].u_mem.ar_fire; s.r_beats = g_ddr[3].u_mem.r_beats;
                s.busy_cyc = g_ddr[3].u_mem.busy_cyc;
            end
            default: s = '{default:0};
        endcase
    endtask

    // ----------------- 比对 final OFM -----------------
    task automatic check_final_ofb(input string case_dir, output int mismatches);
        string final_dir;
        int n_words;
        longint ofb_addr_full;
        logic [NUM_COL*DATA_WIDTH-1:0] expected, got;
        n_words = layer_ofb_words[n_layers - 1];
        final_dir = layer_dirs[n_layers - 1];
        if (final_dir == "") final_dir = $sformatf("%s/chain_data/layer%02d", case_dir, n_layers - 1);
        $readmemh($sformatf("%s/expected_ofm.txt", final_dir), exp_arr);
        mismatches = 0;
        if (multi_ddr_mode) begin
            // PoC 4-DDR: 每核 ODMA 写自己 slot 的 W 段, full image stitch 复杂.
            // 这里跳过 bit-exact 比对, 只关心 cycles (带宽收益是 PoC 主目标).
            $display("  [4-DDR PoC] OFB bit-exact check SKIPPED — see cycles for BW comparison.");
            mismatches = 0;
        end else begin
            ofb_addr_full = ddr_final_ofm_base;
            for (int i = 0; i < n_words; i++) begin
                expected = exp_arr[i];
                got      = ddr_read_addr(ofb_addr_full, i);
                if (got !== expected) begin
                    if (mismatches < 5)
                        $display("  FINAL.OFB[%0d]: expect=%h got=%h", i, expected, got);
                    mismatches++;
                end
            end
        end
    endtask

    // ----------------- 主流程 -----------------
    initial begin
        string  case_dir;
        string  ldir;
        int     mismatches;
        longint t_start;
        longint t_layer_start;
        logic [NUM_CORES-1:0] expected_done_mask;
        // Profiling 快照: 起点 / 每层开始 / 每核当前
        core_snap_t snap_core_t0  [NUM_CORES];   // 整个 run 起点
        core_snap_t snap_core_lst [NUM_CORES];   // 当前 layer 起点
        core_snap_t snap_core_now [NUM_CORES];   // 临时当前
        ddr_snap_t  snap_ddr_t0   [NUM_DDR];
        ddr_snap_t  snap_ddr_lst  [NUM_DDR];
        ddr_snap_t  snap_ddr_now  [NUM_DDR];
        int         layer_cycles;

        case_dir = "cases/multicore_demo";

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_multicore_chain: multi-layer chain N=%0d (host stage barrier) ==", NUM_CORES);

        parse_meta(case_dir);

        // 各层数据 preload (IFB/WB/RDMA, 第一层 IFB 是整网入口, 中间层 IFB 由上层 OFM 写)
        for (int l = 0; l < n_layers; l++) begin
            ldir = layer_dirs[l];
            if (ldir == "") ldir = $sformatf("%s/chain_data/layer%02d", case_dir, l);
            if (l == 0 || layer_preload_ifb[l] == 1)
                preload_ifb(ldir, layer_ddr_ifb[l], layer_ifb_words[l]);
            preload_wb(ldir, layer_ddr_wb[l], layer_wb_words[l]);
            preload_rdma(ldir, layer_ddr_rdma[l], layer_rdma_words[l]);
            // Per-(core, layer) sliced RDMA (residual + W slice)
            for (int c = 0; c < NUM_CORES; c++)
                if (core_layer_rdma_words[c][l] > 0)
                    preload_core_layer_rdma(ldir, c, core_layer_rdma_base[c][l],
                                              core_layer_rdma_words[c][l]);
        end

        // 各核每层 desc list 都 preload (host stage barrier 模式: 每层 host 切换 DESC_LIST_BASE)
        for (int c = 0; c < NUM_CORES; c++)
            for (int l = 0; l < n_layers; l++)
                if (core_layer_desc_count[c][l] > 0)
                    preload_layer_desc(case_dir, c, l, core_layer_desc_base[c][l]);

        // 清 final OFM 区 (multi_ddr 时清 4 slot 同位置)
        if (multi_ddr_mode) begin
            automatic int wbase = (ddr_final_ofm_base & 32'h0FFFFFFF) / 16;
            for (int s = 0; s < 4; s++)
                for (int i = 0; i < layer_ofb_words[n_layers - 1]; i++)
                    ddr_write_slot(s, wbase + i, '0);
        end else begin
            for (int i = 0; i < layer_ofb_words[n_layers - 1]; i++)
                ddr_write_addr(ddr_final_ofm_base, i, '0);
        end

        $display("  DDR preload done. Stage barrier loop start.");
        t_start = $time;
        // 整个 run 起点快照 (per-core / per-DDR)
        for (int c = 0; c < NUM_CORES; c++) snap_core(c, snap_core_t0[c]);
        for (int d = 0; d < NUM_DDR;   d++) snap_ddr (d, snap_ddr_t0 [d]);

        // ---- 主 stage barrier 循环: host 一层一层串 ----
        for (int l = 0; l < n_layers; l++) begin
            t_layer_start = $time;
            // per-layer 起点快照
            for (int c = 0; c < NUM_CORES; c++) snap_core(c, snap_core_lst[c]);
            for (int d = 0; d < NUM_DDR;   d++) snap_ddr (d, snap_ddr_lst [d]);

            // 哪些 core 参与本层 (count > 0 即参与)
            expected_done_mask = '0;
            for (int c = 0; c < NUM_CORES; c++)
                if (core_layer_desc_count[c][l] > 0)
                    expected_done_mask[c] = 1'b1;

            // 1) 每核串行: 写 boot regs + start_dfe + 等 dfe_done.
            //   并行触发 dfe 会让 dfe_busy 1→0 错过 wait, 串行最稳 (DFE ~1000 cy 短开销).
            for (int c = 0; c < NUM_CORES; c++) begin
                if (core_layer_desc_count[c][l] > 0) begin
                    axi_lite_write(CORE_ID_W'(c), ADDR_DESC_LIST_BASE, core_layer_desc_base[c][l]);
                    axi_lite_write(CORE_ID_W'(c), ADDR_DESC_COUNT,     core_layer_desc_count[c][l]);
                    axi_lite_write_dfe_start(CORE_ID_W'(c));
                    wait_dfe_done(CORE_ID_W'(c));
                end
            end

            // 2) 所有 desc 都进 FIFO 了, 现在并行触发 start_layer (clear sticky + 启计算)
            for (int c = 0; c < NUM_CORES; c++)
                if (core_layer_desc_count[c][l] > 0)
                    axi_lite_write_layer_start(CORE_ID_W'(c));

            // 3) 等所有参与 core 的 done_per_core (= core_done_sticky)
            wait ((done_per_core & expected_done_mask) == expected_done_mask);
            layer_cycles = ($time - t_layer_start) / 10;
            $display("  Layer %0d done @ t=%0t (cycles=%0d, mask=%b)",
                     l, $time, layer_cycles, expected_done_mask);

            @(posedge clk); @(posedge clk);

            // ---- LAYER_PROFILE: per-layer per-core MAC fire + per-DDR busy ----
            for (int c = 0; c < NUM_CORES; c++) snap_core(c, snap_core_now[c]);
            for (int d = 0; d < NUM_DDR;   d++) snap_ddr (d, snap_ddr_now [d]);
            begin
                automatic int total_mac_fire = 0;
                automatic int max_ddr_busy   = 0;
                automatic int sum_ddr_busy   = 0;
                automatic int active_cores   = 0;
                for (int c = 0; c < NUM_CORES; c++)
                    if (expected_done_mask[c]) begin
                        total_mac_fire += (snap_core_now[c].mac_fire - snap_core_lst[c].mac_fire);
                        active_cores++;
                    end
                for (int d = 0; d < NUM_DDR; d++) begin
                    automatic int busy_d = snap_ddr_now[d].busy_cyc - snap_ddr_lst[d].busy_cyc;
                    sum_ddr_busy += busy_d;
                    if (busy_d > max_ddr_busy) max_ddr_busy = busy_d;
                end
                // mac_pipe_pct: per-core MAC stage-0 join 成功的周期占比平均值
                //   单核 peak = cycles (mac_fire 是周期数, 不是 ops 数 — 跟 tb_core_dma 的
                //   mac_util(=ops/(cy*256)) 不同口径; 这里反映 pipe 利用率)
                $display("LAYER_PROFILE l=%0d cores=%b cycles=%0d active_cores=%0d total_mac_fire=%0d mac_pipe_pct=%.2f%% ddr_busy_max=%0d (%.1f%%) ddr_busy_sum=%0d (%.1f%% of N*cycles)",
                         l, expected_done_mask, layer_cycles, active_cores, total_mac_fire,
                         (active_cores == 0) ? 0.0 :
                            (real'(total_mac_fire) / (real'(layer_cycles) * real'(active_cores))) * 100.0,
                         max_ddr_busy,
                         (layer_cycles == 0) ? 0.0 : (real'(max_ddr_busy) / real'(layer_cycles)) * 100.0,
                         sum_ddr_busy,
                         (layer_cycles == 0) ? 0.0 : (real'(sum_ddr_busy) / (real'(layer_cycles) * real'(NUM_DDR))) * 100.0);
                // per-core delta (compact)
                for (int c = 0; c < NUM_CORES; c++)
                    if (expected_done_mask[c])
                        $display("  LAYER_CORE l=%0d c=%0d mac_fire=%0d act_fire=%0d act_stall=%0d act_idle=%0d wgt_stall=%0d psum_stall=%0d acc_stall=%0d",
                                 l, c,
                                 snap_core_now[c].mac_fire   - snap_core_lst[c].mac_fire,
                                 snap_core_now[c].act_fire   - snap_core_lst[c].act_fire,
                                 snap_core_now[c].act_stall  - snap_core_lst[c].act_stall,
                                 snap_core_now[c].act_idle   - snap_core_lst[c].act_idle,
                                 snap_core_now[c].wgt_stall  - snap_core_lst[c].wgt_stall,
                                 snap_core_now[c].psum_stall - snap_core_lst[c].psum_stall,
                                 snap_core_now[c].acc_stall  - snap_core_lst[c].acc_stall);
                // per-DDR delta (compact)
                for (int d = 0; d < NUM_DDR; d++)
                    $display("  LAYER_DDR   l=%0d d=%0d aw_fire=%0d w_beats=%0d ar_fire=%0d r_beats=%0d busy_cyc=%0d busy_pct=%.1f%%",
                             l, d,
                             snap_ddr_now[d].aw_fire  - snap_ddr_lst[d].aw_fire,
                             snap_ddr_now[d].w_beats  - snap_ddr_lst[d].w_beats,
                             snap_ddr_now[d].ar_fire  - snap_ddr_lst[d].ar_fire,
                             snap_ddr_now[d].r_beats  - snap_ddr_lst[d].r_beats,
                             snap_ddr_now[d].busy_cyc - snap_ddr_lst[d].busy_cyc,
                             (layer_cycles == 0) ? 0.0 :
                                (real'(snap_ddr_now[d].busy_cyc - snap_ddr_lst[d].busy_cyc) / real'(layer_cycles)) * 100.0);
            end

            // 每 layer 完成后立刻 check 自己的 OFM (intermediate or final).
            // multi_ddr_mode 下 OFM 分散在 4 slot, 跳过 bit-exact 只看 cycles.
            if (multi_ddr_mode) begin
                $display("    POST-LAYER %0d OFM check SKIPPED (multi_ddr cycles-only)", l);
            end else begin
                automatic string ldir2;
                automatic int    n_w2;
                automatic int    mm2;
                automatic longint ofb_addr;
                automatic logic [NUM_COL*DATA_WIDTH-1:0] expected2, got2;
                n_w2 = layer_ofb_words[l];
                mm2  = 0;
                ldir2 = layer_dirs[l];
                if (ldir2 == "") ldir2 = $sformatf("%s/chain_data/layer%02d", case_dir, l);
                if (l == n_layers - 1) ofb_addr = ddr_final_ofm_base;
                else                   ofb_addr = layer_ddr_ofb[l];
                $readmemh($sformatf("%s/expected_ofm.txt", ldir2), exp_arr);
                for (int i = 0; i < n_w2; i++) begin
                    expected2 = exp_arr[i];
                    got2      = ddr_read_addr(ofb_addr, i)[NUM_COL*DATA_WIDTH-1:0];
                    if (got2 !== expected2) mm2++;
                end
                $display("    POST-LAYER %0d OFM check @ 0x%h: mismatches=%0d/%0d",
                         l, ofb_addr, mm2, n_w2);
            end
        end

        $display("  ALL layers done @ t=%0t (total cycles=%0d)", $time, ($time - t_start) / 10);

        // DEBUG: 比对每层中间 OFM (multi_ddr 跳过, OFM 分散在 4 slot)
        if (!multi_ddr_mode) begin
            string ldir;
            int    n_w;
            int    mm;
            logic [NUM_COL*DATA_WIDTH-1:0] expected, got;
            for (int l = 0; l < n_layers - 1; l++) begin
                n_w = layer_ofb_words[l];
                mm = 0;
                ldir = layer_dirs[l];
                if (ldir == "") ldir = $sformatf("%s/chain_data/layer%02d", case_dir, l);
                $readmemh($sformatf("%s/expected_ofm.txt", ldir), exp_arr);
                for (int i = 0; i < n_w; i++) begin
                    expected = exp_arr[i];
                    got      = ddr_read_addr(layer_ddr_ofb[l], i)[NUM_COL*DATA_WIDTH-1:0];
                    if (got !== expected) mm++;
                end
                $display("  Layer %0d intermediate OFM @ 0x%h: mismatches=%0d/%0d  first got=%h vs exp=%h",
                         l, layer_ddr_ofb[l], mm, n_w,
                         ddr_read_addr(layer_ddr_ofb[l], 0)[NUM_COL*DATA_WIDTH-1:0],
                         exp_arr[0]);
            end
        end


        check_final_ofb(case_dir, mismatches);

        // ---- 整网 CORE_RESULT / DDR_PROFILE (whole run delta) ----
        for (int c = 0; c < NUM_CORES; c++) snap_core(c, snap_core_now[c]);
        for (int d = 0; d < NUM_DDR;   d++) snap_ddr (d, snap_ddr_now [d]);
        begin
            automatic int total_cycles = ($time - t_start) / 10;
            automatic string verdict = (mismatches == 0) ? "PASS" : "FAIL";
            for (int c = 0; c < NUM_CORES; c++) begin
                automatic int dmac   = snap_core_now[c].mac_fire - snap_core_t0[c].mac_fire;
                automatic int darfw  = snap_core_now[c].arf_w    - snap_core_t0[c].arf_w;
                automatic int darfr  = snap_core_now[c].arf_r    - snap_core_t0[c].arf_r;
                automatic int dparff = snap_core_now[c].parf_f   - snap_core_t0[c].parf_f;
                automatic int dparfd = snap_core_now[c].parf_d   - snap_core_t0[c].parf_d;
                $display("CORE_RESULT c=%0d %s cycles=%0d mac_fire=%0d mac_pipe_pct=%.2f%% arf_w=%0d arf_r=%0d parf_f=%0d parf_d=%0d",
                         c, verdict, total_cycles, dmac,
                         (real'(dmac) / real'(total_cycles)) * 100.0,
                         darfw, darfr, dparff, dparfd);
            end
            for (int c = 0; c < NUM_CORES; c++)
                $display("CORE_PROFILE c=%0d cycles=%0d act_fire=%0d act_stall=%0d act_idle=%0d wgt_fire=%0d wgt_stall=%0d wgt_idle=%0d psum_fire=%0d psum_stall=%0d psum_idle=%0d acc_fire=%0d acc_stall=%0d acc_idle=%0d",
                         c, total_cycles,
                         snap_core_now[c].act_fire   - snap_core_t0[c].act_fire,
                         snap_core_now[c].act_stall  - snap_core_t0[c].act_stall,
                         snap_core_now[c].act_idle   - snap_core_t0[c].act_idle,
                         snap_core_now[c].wgt_fire   - snap_core_t0[c].wgt_fire,
                         snap_core_now[c].wgt_stall  - snap_core_t0[c].wgt_stall,
                         snap_core_now[c].wgt_idle   - snap_core_t0[c].wgt_idle,
                         snap_core_now[c].psum_fire  - snap_core_t0[c].psum_fire,
                         snap_core_now[c].psum_stall - snap_core_t0[c].psum_stall,
                         snap_core_now[c].psum_idle  - snap_core_t0[c].psum_idle,
                         snap_core_now[c].acc_fire   - snap_core_t0[c].acc_fire,
                         snap_core_now[c].acc_stall  - snap_core_t0[c].acc_stall,
                         snap_core_now[c].acc_idle   - snap_core_t0[c].acc_idle);
            for (int d = 0; d < NUM_DDR; d++) begin
                automatic int daw   = snap_ddr_now[d].aw_fire  - snap_ddr_t0[d].aw_fire;
                automatic int dwb   = snap_ddr_now[d].w_beats  - snap_ddr_t0[d].w_beats;
                automatic int dar   = snap_ddr_now[d].ar_fire  - snap_ddr_t0[d].ar_fire;
                automatic int drb   = snap_ddr_now[d].r_beats  - snap_ddr_t0[d].r_beats;
                automatic int dbusy = snap_ddr_now[d].busy_cyc - snap_ddr_t0[d].busy_cyc;
                $display("DDR_PROFILE d=%0d cycles=%0d aw_fire=%0d w_beats=%0d ar_fire=%0d r_beats=%0d busy_cyc=%0d busy_pct=%.2f%%",
                         d, total_cycles, daw, dwb, dar, drb, dbusy,
                         (real'(dbusy) / real'(total_cycles)) * 100.0);
            end
        end

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

    // 总 watchdog (ResNet11 Patch ~8M cycles + 11 层 ~50M total at 10ns/cy = 500M ns 安全余量)
    initial begin
        #1_000_000_000;
        $display("FATAL: watchdog timeout @ %0t", $time);
        $stop;
    end

endmodule
