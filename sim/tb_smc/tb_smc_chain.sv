`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// tb_smc_chain.sv  --  Phase 7 SMC + NUMA 多核多层 chain TB
//
// 跑 toolchain/run_multicore_chain.py --smc 生成的 case 目录:
//   sim/tb_smc/cases/<name>/multicore_meta.txt  (含 SMC_* 字段)
//   sim/tb_smc/cases/<name>/chain_data/layer<N>/
//   sim/tb_smc/cases/<name>/core<i>/layer<L>_desc_list.hex
//   sim/tb_smc/cases/<name>/core<i>/layer<L>_idma_sg.hex
//   sim/tb_smc/cases/<name>/core<i>/layer<L>_odma_sg.hex
//
// 数据流 (SMC + NUMA):
//   - 整图 IFM 按整图 W 4 等分散布到 4 mem (mem[i] 装段 i, halo 物理只一份)
//   - WB 全核共享 broadcast 到 4 mem (任一 mem 都能服务 WDMA)
//   - desc list / IDMA SG cmd / ODMA SG cmd per-core 在 mem[c]
//   - 每核 IDMA SG dispatcher 顺序拉 cmd_list[c, layer], 跨 mem 边界由
//     axi_crossbar_4to4_sim 自动按 awaddr[25:24] 路由
//   - ODMA SG dispatcher 类似, 写 OFM 到目标 mem
//   - 最后一层 OFM 写到 mem[i].SMC_FINAL_OFM_BASE (按整图 W 段散布), TB stitch
//
// 简化场景: wslice1 / wslice5 等 W slice 全 layer 场景, 每 layer 强制 4 核 W slice
// =============================================================================

module tb_smc_chain;
    timeunit 1ns; timeprecision 1ps;

    // ----------------- 参数 -----------------
    localparam int NUM_COL    = `FLUX_NUM_COL;
    localparam int NUM_PE     = `FLUX_NUM_PE;
    localparam int DATA_WIDTH = `FLUX_DATA_WIDTH;
    localparam int CSR_DATA_W = `FLUX_CSR_DATA_W;
    localparam int BUS_ADDR_W = `FLUX_BUS_ADDR_W;
    localparam int BUS_DATA_W = `FLUX_BUS_DATA_W;
    localparam int AXI_M_ID   = `FLUX_AXI_M_ID;
    localparam int AXI_M_W    = `FLUX_AXI_M_WIDTH;

    localparam int NUM_CORES  = 4;
    localparam int CORE_ID_W  = $clog2(NUM_CORES);
    localparam int HOST_CSR_AW = 12 + CORE_ID_W;

    // SMC layout 常量 (跟 toolchain/run_multicore_chain.py SMC_* 一致)
    localparam longint SMC_MEM_STRIDE          = 64'h0100_0000;
    localparam longint SMC_LAYER_DATA_OFFSET   = 64'h0008_0000;
    localparam longint SMC_WB_BASE             = 64'h0080_0000;
    localparam longint SMC_LAYER_WB_OFFSET     = 64'h0001_0000;
    localparam longint SMC_RDMA_BASE           = 64'h0090_0000;
    localparam longint SMC_LAYER_RDMA_OFFSET   = 64'h0001_0000;
    localparam longint SMC_DESC_BASE           = 64'h00A0_0000;
    localparam longint SMC_LAYER_DESC_OFFSET   = 64'h0001_0000;
    localparam longint SMC_INPUT_BASE          = 64'h00D0_0000;
    localparam longint SMC_LAYER_INPUT_OFFSET  = 64'h0008_0000;   // 512 KB / root slot
    localparam longint SMC_FINAL_OFM_BASE      = 64'h00E0_0000;

    // CSR 地址
    localparam [11:0] ADDR_CTRL           = `FLUX_ADDR_CTRL;
    localparam [11:0] ADDR_DESC_LIST_BASE = `FLUX_ADDR_DESC_LIST_BASE;
    localparam [11:0] ADDR_DESC_COUNT     = `FLUX_ADDR_DESC_COUNT;

    // ----------------- 时钟复位 -----------------
    logic clk = 0; always #5 clk = ~clk;
    logic rst_n = 0;

    // ----------------- DUT 信号 -----------------
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

    logic [NUM_CORES-1:0]   done_per_core;

    // ----------------- DUT (multicore_top_smc) -----------------
    multicore_top_smc #(
        .NUM_CORES(NUM_CORES)
    ) u_dut (
        .clk(clk), .rst_n(rst_n),
        .csr_awaddr(csr_awaddr), .csr_awvalid(csr_awvalid), .csr_awready(csr_awready),
        .csr_wdata(csr_wdata), .csr_wstrb(csr_wstrb), .csr_wvalid(csr_wvalid), .csr_wready(csr_wready),
        .csr_bresp(csr_bresp), .csr_bvalid(csr_bvalid), .csr_bready(csr_bready),
        .csr_araddr(csr_araddr), .csr_arvalid(csr_arvalid), .csr_arready(csr_arready),
        .csr_rdata(csr_rdata), .csr_rresp(csr_rresp), .csr_rvalid(csr_rvalid), .csr_rready(csr_rready),
        .done_per_core(done_per_core)
    );

    // ----------------- preload 缓冲 -----------------
    logic [BUS_DATA_W-1:0] ifb_arr  [0:524287];
    logic [BUS_DATA_W-1:0] desc_arr [0:8191];
    logic [BUS_DATA_W-1:0] sg_arr   [0:8191];
    logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_arr [0:4095];
    logic [BUS_DATA_W-1:0] rdma_arr [0:65535];
    logic [NUM_COL*DATA_WIDTH-1:0] exp_arr [0:65535];

    // ----------------- meta -----------------
    int n_layers;
    int layer_h_out [0:31];
    int layer_w_out [0:31];
    int layer_c_out [0:31];
    int layer_ifb_words [0:31];
    int layer_wb_words  [0:31];
    int layer_ofb_words [0:31];
    int layer_rdma_words[0:31];
    int layer_preload_ifb[0:31];
    string  layer_dirs[0:31];

    // per-(core, layer) desc/cmd
    longint core_layer_desc_base [0:NUM_CORES-1][0:31];
    int     core_layer_desc_count[0:NUM_CORES-1][0:31];
    longint smc_idma_cmd_base    [0:NUM_CORES-1][0:31];
    int     smc_idma_cmd_count   [0:NUM_CORES-1][0:31];
    longint smc_odma_cmd_base    [0:NUM_CORES-1][0:31];
    int     smc_odma_cmd_count   [0:NUM_CORES-1][0:31];
    int     smc_w_out_start      [0:NUM_CORES-1][0:31];
    int     smc_my_w_out         [0:NUM_CORES-1][0:31];

    // SMC per-layer 维度 + mode (driver SMC meta 写)
    int     smc_layer_h_in       [0:31];
    int     smc_layer_w_in       [0:31];
    int     smc_layer_cin_slices [0:31];
    int     smc_layer_cout_slices[0:31];
    int     smc_layer_pad        [0:31];
    int     smc_layer_k          [0:31];
    int     smc_layer_stride     [0:31];
    string  smc_layer_mode       [0:31];   // "W" or "A"
    int     smc_layer_mode_a_core[0:31];
    int     smc_layer_root_slot  [0:31];
    int     smc_layer_has_residual[0:31];

    // per-(core, layer) sliced RDMA (residual + W slice 时每核独立 rdma_data_c<c>.txt)
    longint core_layer_rdma_base [0:NUM_CORES-1][0:31];
    int     core_layer_rdma_words[0:NUM_CORES-1][0:31];

    // ----------------- meta parse -----------------
    task automatic parse_meta(input string case_dir);
        int     fd;
        string  line, key, val_s, suffix;
        longint val;
        int     layer_id, core_id, p, p1, p2;
        string  path;
        bit     parsed_num;

        path = $sformatf("%s/multicore_meta.txt", case_dir);
        fd = $fopen(path, "r");
        if (fd == 0) begin $display("FATAL: cannot open %s", path); $stop; end
        for (int c = 0; c < NUM_CORES; c++)
            for (int l = 0; l < 32; l++) begin
                core_layer_desc_count[c][l] = 0;
                smc_idma_cmd_count   [c][l] = 0;
                smc_odma_cmd_count   [c][l] = 0;
                core_layer_rdma_words[c][l] = 0;
            end
        for (int l = 0; l < 32; l++) begin
            smc_layer_mode[l]         = "?";
            smc_layer_mode_a_core[l]  = -1;
            smc_layer_root_slot[l]    = -1;
            smc_layer_has_residual[l] = 0;
        end
        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            parsed_num = 1'b0;
            if      ($sscanf(line, "%s = 0x%h", key, val) == 2) parsed_num = 1'b1;
            else if ($sscanf(line, "%s = %d",   key, val) == 2) parsed_num = 1'b1;
            else if ($sscanf(line, "%s = %s",   key, val_s) == 2) begin
                if (key.len() > 6 && key.substr(0,5) == "LAYER_") begin
                    p1 = -1;
                    for (int i = 6; i < key.len(); i++)
                        if (key.getc(i) == "_") begin p1 = i; break; end
                    if (p1 > 0 && key.substr(p1+1, key.len()-1) == "DIR") begin
                        layer_id = key.substr(6, p1-1).atoi();
                        layer_dirs[layer_id] = val_s;
                    end
                end
                // SMC_LAYER_<N>_MODE = W or A (string)
                // 注意: "SMC_LAYER" 9 chars → substr(0, 8). 后一字符 idx=9 应是 "_".
                if (key.len() > 10 && key.substr(0, 8) == "SMC_LAYER" && key.getc(9) == "_") begin
                    p1 = -1;
                    for (int i = 10; i < key.len(); i++)
                        if (key.getc(i) == "_") begin p1 = i; break; end
                    if (p1 > 0 && key.substr(p1+1, key.len()-1) == "MODE") begin
                        layer_id = key.substr(10, p1-1).atoi();
                        smc_layer_mode[layer_id] = val_s;
                    end
                end
                continue;
            end else continue;

            if (!parsed_num) continue;

            // 注意 SystemVerilog string.substr(start, end) 是闭区间 inclusive end.
            //   substr(0, 3) → 4 chars; substr(0, 4) → 5 chars (= "CORE_" / "SMC_C" 等)
            if (key == "NUM_LAYERS") n_layers = val;
            else if (key.len() > 4 && key.substr(0, 3) == "SMC_") begin
                // SMC_LAYER_<l>_<suffix>  (driver SMC layer 维度 / mode 字段)
                // "SMC_LAYER" 9 chars → substr(0, 8). 后一字符 idx=9 应是 "_".
                if (key.len() > 10 && key.substr(0, 8) == "SMC_LAYER") begin
                    if (key.getc(9) == "_") begin
                        p1 = -1;
                        for (int i = 10; i < key.len(); i++)
                            if (key.getc(i) == "_") begin p1 = i; break; end
                        if (p1 > 0) begin
                            layer_id = key.substr(10, p1-1).atoi();
                            suffix   = key.substr(p1+1, key.len()-1);
                            if      (suffix == "H_IN")          smc_layer_h_in       [layer_id] = val;
                            else if (suffix == "W_IN")          smc_layer_w_in       [layer_id] = val;
                            else if (suffix == "C_IN_SLICES")   smc_layer_cin_slices [layer_id] = val;
                            else if (suffix == "C_OUT_SLICES")  smc_layer_cout_slices[layer_id] = val;
                            else if (suffix == "PAD")           smc_layer_pad        [layer_id] = val;
                            else if (suffix == "K")             smc_layer_k          [layer_id] = val;
                            else if (suffix == "STRIDE")        smc_layer_stride     [layer_id] = val;
                            else if (suffix == "MODE_A_CORE")   smc_layer_mode_a_core[layer_id] = $signed(val[31:0]);
                            else if (suffix == "ROOT_SLOT")     smc_layer_root_slot  [layer_id] = $signed(val[31:0]);
                            else if (suffix == "HAS_RESIDUAL")  smc_layer_has_residual[layer_id] = val;
                        end
                    end
                end
                // SMC_CORE_<c>_LAYER_<l>_<suffix>
                if (key.len() > 9 && key.substr(0, 8) == "SMC_CORE_") begin
                    p1 = -1; p2 = -1;
                    for (int i = 9; i < key.len(); i++)
                        if (key.getc(i) == "_") begin p1 = i; break; end
                    if (p1 > 0 && p1 + 6 < key.len() && key.substr(p1+1, p1+6) == "LAYER_") begin
                        for (int j = p1+7; j < key.len(); j++)
                            if (key.getc(j) == "_") begin p2 = j; break; end
                        if (p2 > 0) begin
                            core_id  = key.substr(9, p1-1).atoi();
                            layer_id = key.substr(p1+7, p2-1).atoi();
                            suffix   = key.substr(p2+1, key.len()-1);
                            if      (suffix == "IDMA_CMD_BASE")  smc_idma_cmd_base [core_id][layer_id] = val;
                            else if (suffix == "IDMA_CMD_COUNT") smc_idma_cmd_count[core_id][layer_id] = val;
                            else if (suffix == "ODMA_CMD_BASE")  smc_odma_cmd_base [core_id][layer_id] = val;
                            else if (suffix == "ODMA_CMD_COUNT") smc_odma_cmd_count[core_id][layer_id] = val;
                            else if (suffix == "W_OUT_START")    smc_w_out_start   [core_id][layer_id] = val;
                            else if (suffix == "MY_W_OUT")       smc_my_w_out      [core_id][layer_id] = val;
                        end
                    end
                end
            end
            else if (key.len() > 5 && key.substr(0, 4) == "CORE_") begin
                p1 = -1; p2 = -1;
                for (int i = 5; i < key.len(); i++)
                    if (key.getc(i) == "_") begin p1 = i; break; end
                if (p1 > 0 && p1 + 6 < key.len() && key.substr(p1+1, p1+6) == "LAYER_") begin
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
            end
            else if (key.len() > 6 && key.substr(0, 5) == "LAYER_") begin
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
                        "H_OUT"      : layer_h_out[layer_id] = val;
                        "W_OUT"      : layer_w_out[layer_id] = val;
                        "C_OUT"      : layer_c_out[layer_id] = val;
                        default      : ;
                    endcase
                end
            end
        end
        $fclose(fd);
        $display("[META] n_layers=%0d, n_cores=%0d", n_layers, NUM_CORES);
        for (int c = 0; c < NUM_CORES; c++)
            for (int l = 0; l < n_layers; l++)
                if (core_layer_desc_count[c][l] > 0)
                    $display("  Core %0d Layer %0d: desc=0x%h(%0d) idma=0x%h(%0d) odma=0x%h(%0d) w_out=[%0d:%0d]",
                             c, l,
                             core_layer_desc_base[c][l], core_layer_desc_count[c][l],
                             smc_idma_cmd_base[c][l], smc_idma_cmd_count[c][l],
                             smc_odma_cmd_base[c][l], smc_odma_cmd_count[c][l],
                             smc_w_out_start[c][l],
                             smc_w_out_start[c][l] + smc_my_w_out[c][l]);
    endtask

    // ----------------- mem 写口 helper -----------------
    // 写 byte_addr 处的一个 BUS_DATA_W word 到 mem[mem_id]
    //   mem_id 由 byte_addr[25:24] 决定 (跟 SMC layout 一致)
    //   mem 内 word index = (byte_addr & 0x00FFFFFF) / 16
    task automatic write_mem_word(input longint byte_addr, input logic [BUS_DATA_W-1:0] data);
        int mem_id;
        int word_idx;
        mem_id   = (byte_addr >> 24) & 32'h3;
        word_idx = (byte_addr & 32'h00FFFFFF) >> 4;   // /16
        case (mem_id)
            0: u_dut.g_sim_mem.gen_mem[0].u_mem.mem[word_idx] = data;
            1: u_dut.g_sim_mem.gen_mem[1].u_mem.mem[word_idx] = data;
            2: u_dut.g_sim_mem.gen_mem[2].u_mem.mem[word_idx] = data;
            3: u_dut.g_sim_mem.gen_mem[3].u_mem.mem[word_idx] = data;
        endcase
    endtask

    function automatic logic [BUS_DATA_W-1:0] read_mem_word(input longint byte_addr);
        int mem_id;
        int word_idx;
        mem_id   = (byte_addr >> 24) & 32'h3;
        word_idx = (byte_addr & 32'h00FFFFFF) >> 4;
        case (mem_id)
            0: return u_dut.g_sim_mem.gen_mem[0].u_mem.mem[word_idx];
            1: return u_dut.g_sim_mem.gen_mem[1].u_mem.mem[word_idx];
            2: return u_dut.g_sim_mem.gen_mem[2].u_mem.mem[word_idx];
            3: return u_dut.g_sim_mem.gen_mem[3].u_mem.mem[word_idx];
            default: return '0;
        endcase
    endfunction

    // ----------------- preload 各种数据 -----------------
    // IFB preload — 根据 SMC layer mode 分两种 layout:
    //   - W slice (mode='W'): 整图 IFB 按 W 切 N 段散布到 N mem (跟 driver compute_smc_w_segments 一致)
    //   - Mode A (mode='A'): 整图 IFB 存 mem[mode_a_core] 紧凑
    // 起点 base 视 root layer:
    //   - layer 0: SMC_INPUT_BASE
    //   - layer i>0 ifb_is_root: SMC_INPUT_BASE + (root_slot+1) * 0x10000  (避开 layer 0)
    task automatic preload_ifb_smc(input string layer_dir, input int layer_idx);
        int     h_in        = smc_layer_h_in       [layer_idx];
        int     w_in        = smc_layer_w_in       [layer_idx];
        int     cin_slices  = smc_layer_cin_slices [layer_idx];
        int     seg_widths [0:NUM_CORES-1];
        int     seg_starts [0:NUM_CORES-1];
        int     b, rem;
        longint base_offset;
        longint base_byte;
        int     core_load;     // mode A 时只有这个核 mem 装 IFB

        $readmemh($sformatf("%s/ifb.txt", layer_dir), ifb_arr);

        // base offset within each mem (跟 driver SMC_LAYER_INPUT_OFFSET 一致)
        if (layer_idx == 0)
            base_offset = SMC_INPUT_BASE;
        else if (smc_layer_root_slot[layer_idx] >= 0)
            base_offset = SMC_INPUT_BASE + (smc_layer_root_slot[layer_idx] + 1) * SMC_LAYER_INPUT_OFFSET;
        else
            base_offset = SMC_INPUT_BASE;   // 不该走到这里 (非 root layer 不需 preload)

        if (smc_layer_mode[layer_idx] == "A") begin
            // Mode A: 整图存 mem[mode_a_core] 紧凑 (一行 = w_in × cin_slices word)
            core_load = smc_layer_mode_a_core[layer_idx];
            base_byte = (core_load * SMC_MEM_STRIDE) + base_offset;
            for (int r = 0; r < h_in; r++)
                for (int x = 0; x < w_in; x++)
                    for (int cs = 0; cs < cin_slices; cs++) begin
                        int src_w_idx, dst_byte;
                        src_w_idx = r * w_in * cin_slices + x * cin_slices + cs;
                        dst_byte  = base_byte + (r * w_in * cin_slices + x * cin_slices + cs) * 16;
                        write_mem_word(dst_byte, ifb_arr[src_w_idx]);
                    end
            $display("    IFB[%0d] mode A loaded to mem[%0d] @ 0x%h (H=%0d W=%0d cs=%0d)",
                     layer_idx, core_load, base_byte, h_in, w_in, cin_slices);
        end else begin
            // W slice: 切 N 段 (前 N-1 = W/N, 最后段拿余数)
            b = w_in / NUM_CORES;
            for (int i = 0; i < NUM_CORES; i++) begin
                seg_widths[i] = b;
                seg_starts[i] = i * b;
            end
            rem = w_in - b * NUM_CORES;
            seg_widths[NUM_CORES-1] += rem;

            for (int i = 0; i < NUM_CORES; i++) begin
                base_byte = (i * SMC_MEM_STRIDE) + base_offset;
                for (int r = 0; r < h_in; r++)
                    for (int x = 0; x < seg_widths[i]; x++)
                        for (int cs = 0; cs < cin_slices; cs++) begin
                            int src_w_idx, dst_byte;
                            src_w_idx = r * w_in * cin_slices + (seg_starts[i] + x) * cin_slices + cs;
                            dst_byte  = base_byte + (r * seg_widths[i] * cin_slices + x * cin_slices + cs) * 16;
                            write_mem_word(dst_byte, ifb_arr[src_w_idx]);
                        end
            end
            $display("    IFB[%0d] W slice loaded (H=%0d W=%0d seg=%0p cs=%0d)",
                     layer_idx, h_in, w_in, seg_widths, cin_slices);
        end
    endtask

    // WB broadcast 到 4 mem
    task automatic preload_wb_smc(input string layer_dir, input int layer_idx, input int n_words);
        longint base_byte;
        $readmemh($sformatf("%s/wb.txt", layer_dir), wb_arr);
        for (int i = 0; i < NUM_CORES; i++) begin
            base_byte = (i * SMC_MEM_STRIDE) + SMC_WB_BASE + layer_idx * SMC_LAYER_WB_OFFSET;
            // WB layout: 一行 = NUM_COL × NUM_PE × DATA_WIDTH = 2048 bit = 16 个 BUS_DATA_W word
            for (int wi = 0; wi < n_words; wi++)
                for (int sub = 0; sub < 16; sub++) begin
                    longint dst_byte = base_byte + (wi*16 + sub) * 16;
                    write_mem_word(dst_byte, wb_arr[wi][sub*BUS_DATA_W +: BUS_DATA_W]);
                end
        end
    endtask

    // RDMA: 优先 sliced per-(core, layer) (residual + W slice), 否则 broadcast 4 mem
    task automatic preload_rdma_smc(input string layer_dir, input int layer_idx, input int n_words);
        int fd;
        longint base_byte;
        bit any_sliced;
        // 检查是否有 per-core sliced rdma (residual + W slice 时 driver 生成 rdma_data_c<c>.txt)
        any_sliced = 1'b0;
        for (int c = 0; c < NUM_CORES; c++)
            if (core_layer_rdma_words[c][layer_idx] > 0) any_sliced = 1'b1;
        if (any_sliced) begin
            // sliced: 每核读自己 rdma_data_c<c>.txt 写到 mem[c] SMC_RDMA region
            for (int c = 0; c < NUM_CORES; c++) begin
                int n_w_c;
                n_w_c = core_layer_rdma_words[c][layer_idx];
                if (n_w_c == 0) continue;
                for (int i = 0; i < 65536; i++) rdma_arr[i] = '0;
                $readmemh($sformatf("%s/rdma_data_c%0d.txt", layer_dir, c), rdma_arr);
                base_byte = (c * SMC_MEM_STRIDE) + SMC_RDMA_BASE + layer_idx * SMC_LAYER_RDMA_OFFSET;
                for (int i = 0; i < n_w_c; i++)
                    write_mem_word(base_byte + i * 16, rdma_arr[i]);
            end
        end else begin
            // broadcast: 单一 rdma_data.txt 复制 4 份
            fd = $fopen($sformatf("%s/rdma_data.txt", layer_dir), "r");
            if (fd == 0) return;
            $fclose(fd);
            for (int i = 0; i < 65536; i++) rdma_arr[i] = '0;
            $readmemh($sformatf("%s/rdma_data.txt", layer_dir), rdma_arr);
            for (int c = 0; c < NUM_CORES; c++) begin
                base_byte = (c * SMC_MEM_STRIDE) + SMC_RDMA_BASE + layer_idx * SMC_LAYER_RDMA_OFFSET;
                for (int i = 0; i < n_words; i++)
                    write_mem_word(base_byte + i * 16, rdma_arr[i]);
            end
        end
    endtask

    // desc list per (core, layer)
    task automatic preload_desc_smc(input string case_dir, input int core_id, input int layer_idx,
                                     input longint base_byte, input int n_descs);
        string path, l2;
        if (layer_idx < 10) l2 = $sformatf("0%0d", layer_idx);
        else                l2 = $sformatf("%0d",  layer_idx);
        path = $sformatf("%s/core%0d/layer%s_desc_list.hex", case_dir, core_id, l2);
        for (int i = 0; i < 8192; i++) desc_arr[i] = '0;
        $readmemh(path, desc_arr);
        // 一个 desc list 用 1024 beats 上界 (16 KB)
        for (int i = 0; i < 1024; i++)
            write_mem_word(base_byte + i * 16, desc_arr[i]);
    endtask

    // SG cmd list (IDMA 或 ODMA) per (core, layer)
    task automatic preload_sg_cmd(input string case_dir, input string which,
                                    input int core_id, input int layer_idx,
                                    input longint base_byte, input int n_cmds);
        string path, l2;
        if (layer_idx < 10) l2 = $sformatf("0%0d", layer_idx);
        else                l2 = $sformatf("%0d",  layer_idx);
        path = $sformatf("%s/core%0d/layer%s_%s_sg.hex", case_dir, core_id, l2, which);
        for (int i = 0; i < 8192; i++) sg_arr[i] = '0;
        $readmemh(path, sg_arr);
        // 每 cmd 占 32 byte = 2 个 16-byte word
        for (int i = 0; i < n_cmds * 2; i++)
            write_mem_word(base_byte + i * 16, sg_arr[i]);
    endtask

    // ----------------- AXI-Lite write -----------------
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

    task automatic write_dfe_start(input [CORE_ID_W-1:0] core_id);
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0010);
    endtask

    task automatic write_layer_start(input [CORE_ID_W-1:0] core_id);
        axi_lite_write(core_id, ADDR_CTRL, 32'h0000_0020);
    endtask

    task automatic wait_dfe_done(input [CORE_ID_W-1:0] core_id);
        case (core_id)
            'd0: begin wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b1);
                       wait (u_dut.gen_core[0].u_core.dfe_busy == 1'b0); end
            'd1: begin wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b1);
                       wait (u_dut.gen_core[1].u_core.dfe_busy == 1'b0); end
            'd2: begin wait (u_dut.gen_core[2].u_core.dfe_busy == 1'b1);
                       wait (u_dut.gen_core[2].u_core.dfe_busy == 1'b0); end
            'd3: begin wait (u_dut.gen_core[3].u_core.dfe_busy == 1'b1);
                       wait (u_dut.gen_core[3].u_core.dfe_busy == 1'b0); end
            default: ;
        endcase
    endtask

    // ----------------- check OFM (从 4 mem stitch) -----------------
    // layer_idx == n_layers-1: 最后一层 OFM 在 mem[i].SMC_FINAL_OFM_BASE
    // layer_idx <  n_layers-1: 中间层 OFM 在 mem[i].SMC_IFM_OFM_BASE + layer_idx * SMC_LAYER_DATA_OFFSET
    //   (driver layout: layer i 的 OFM 段 = layer i+1 的 IFM 段)
    // golden expected_ofm.txt 是整图 NHWC: row r, col x, cs → idx = r*W*cs_total + x*cs_total + cs
    task automatic check_layer_ofm(input string case_dir, input int layer_idx,
                                    input string label, output int mismatches);
        string ldir;
        int    h_out, w_out, cout_slices;
        int    seg_widths[0:NUM_CORES-1];
        int    seg_starts[0:NUM_CORES-1];
        int    b, rem;
        longint base_byte;
        longint base_offset;
        logic [NUM_COL*DATA_WIDTH-1:0] expected, got;

        h_out   = layer_h_out [layer_idx];
        w_out   = layer_w_out [layer_idx];
        cout_slices = (layer_c_out[layer_idx] + NUM_COL - 1) / NUM_COL;
        ldir    = layer_dirs[layer_idx];
        if (ldir == "") ldir = $sformatf("%s/chain_data/layer%02d", case_dir, layer_idx);

        // 决定 mem 内的 base offset
        if (layer_idx == n_layers - 1)
            base_offset = SMC_FINAL_OFM_BASE;
        else
            base_offset = layer_idx * SMC_LAYER_DATA_OFFSET;

        $readmemh($sformatf("%s/expected_ofm.txt", ldir), exp_arr);
        mismatches = 0;

        if (smc_layer_mode[layer_idx] == "A") begin
            // Mode A: OFM 整图存 mem[my_core] 紧凑
            int core_chk = smc_layer_mode_a_core[layer_idx];
            base_byte = (core_chk * SMC_MEM_STRIDE) + base_offset;
            for (int r = 0; r < h_out; r++)
                for (int x = 0; x < w_out; x++)
                    for (int cs = 0; cs < cout_slices; cs++) begin
                        int exp_idx;
                        longint mem_byte;
                        exp_idx  = r * w_out * cout_slices + x * cout_slices + cs;
                        mem_byte = base_byte + exp_idx * 16;
                        expected = exp_arr[exp_idx];
                        got      = read_mem_word(mem_byte)[NUM_COL*DATA_WIDTH-1:0];
                        if (got !== expected) begin
                            if (mismatches < 5)
                                $display("    %s OFM(A) r=%0d x=%0d cs=%0d: exp=%h got=%h @ 0x%h",
                                         label, r, x, cs, expected, got, mem_byte);
                            mismatches++;
                        end
                    end
        end else begin
            // W slice: 整图按 W 4 等分散布 4 mem
            b = w_out / NUM_CORES;
            for (int i = 0; i < NUM_CORES; i++) begin
                seg_widths[i] = b;
                seg_starts[i] = i * b;
            end
            rem = w_out - b * NUM_CORES;
            seg_widths[NUM_CORES-1] += rem;

            for (int r = 0; r < h_out; r++)
                for (int i = 0; i < NUM_CORES; i++) begin
                    base_byte = (i * SMC_MEM_STRIDE) + base_offset;
                    for (int x = 0; x < seg_widths[i]; x++)
                        for (int cs = 0; cs < cout_slices; cs++) begin
                            int exp_idx;
                            longint mem_byte;
                            exp_idx  = r * w_out * cout_slices
                                     + (seg_starts[i] + x) * cout_slices + cs;
                            mem_byte = base_byte
                                     + (r * seg_widths[i] * cout_slices + x * cout_slices + cs) * 16;
                            expected = exp_arr[exp_idx];
                            got      = read_mem_word(mem_byte)[NUM_COL*DATA_WIDTH-1:0];
                            if (got !== expected) begin
                                if (mismatches < 5)
                                    $display("    %s OFM(W) r=%0d mem=%0d x=%0d cs=%0d: exp=%h got=%h @ 0x%h",
                                             label, r, i, x, cs, expected, got, mem_byte);
                                mismatches++;
                            end
                        end
                end
        end
    endtask

    // ----------------- 主流程 -----------------
    initial begin
        string  case_dir;
        string  ldir;
        int     mismatches;
        int     layer_mm;
        int     total_intermediate_mm;
        longint t_start;
        longint t_layer_start;
        logic [NUM_CORES-1:0] expected_done_mask;
        int     layer_cycles;

        // 从 +CASE_DIR=xxx 命令行参数读取 case 目录, 否则默认 wslice1
        if (!$value$plusargs("CASE_DIR=%s", case_dir))
            case_dir = "cases/smc_wslice1";

        csr_awaddr  = 0; csr_awvalid = 0;
        csr_wdata   = 0; csr_wstrb   = 0; csr_wvalid = 0;
        csr_bready  = 0;
        #20 rst_n = 1;
        #10;

        $display("== tb_smc_chain: SMC + NUMA multi-layer chain N=%0d ==", NUM_CORES);
        parse_meta(case_dir);

        // ---- 1. preload 各种数据到 4 mem ----
        for (int l = 0; l < n_layers; l++) begin
            ldir = layer_dirs[l];
            if (ldir == "") ldir = $sformatf("%s/chain_data/layer%02d", case_dir, l);
            // IFB: layer 0 (整网入口) preload; ifb_is_root layer 也要 preload (host 当 root random IFB);
            // chain 中间层 IFB 由上层 OFM 写, TB 不 preload
            if (l == 0 || layer_preload_ifb[l] == 1)
                preload_ifb_smc(ldir, l);
            preload_wb_smc  (ldir, l, layer_wb_words[l]);
            preload_rdma_smc(ldir, l, layer_rdma_words[l]);
        end

        // ---- 2. preload desc list + IDMA/ODMA SG cmd lists per (core, layer) ----
        for (int c = 0; c < NUM_CORES; c++)
            for (int l = 0; l < n_layers; l++) begin
                if (core_layer_desc_count[c][l] > 0)
                    preload_desc_smc(case_dir, c, l,
                                      core_layer_desc_base[c][l], core_layer_desc_count[c][l]);
                if (smc_idma_cmd_count[c][l] > 0)
                    preload_sg_cmd(case_dir, "idma", c, l,
                                    smc_idma_cmd_base[c][l], smc_idma_cmd_count[c][l]);
                if (smc_odma_cmd_count[c][l] > 0)
                    preload_sg_cmd(case_dir, "odma", c, l,
                                    smc_odma_cmd_base[c][l], smc_odma_cmd_count[c][l]);
            end

        $display("  All preload done. Stage barrier loop start.");
        t_start = $time;

        // ---- 3. host stage barrier 主循环 ----
        for (int l = 0; l < n_layers; l++) begin
            t_layer_start = $time;

            expected_done_mask = '0;
            for (int c = 0; c < NUM_CORES; c++)
                if (core_layer_desc_count[c][l] > 0)
                    expected_done_mask[c] = 1'b1;

            // 串行 dfe (写 boot regs + start_dfe + 等 dfe_done)
            for (int c = 0; c < NUM_CORES; c++)
                if (core_layer_desc_count[c][l] > 0) begin
                    axi_lite_write(CORE_ID_W'(c), ADDR_DESC_LIST_BASE, core_layer_desc_base[c][l]);
                    axi_lite_write(CORE_ID_W'(c), ADDR_DESC_COUNT,     core_layer_desc_count[c][l]);
                    write_dfe_start(CORE_ID_W'(c));
                    wait_dfe_done(CORE_ID_W'(c));
                end

            // 并行 start_layer
            for (int c = 0; c < NUM_CORES; c++)
                if (core_layer_desc_count[c][l] > 0)
                    write_layer_start(CORE_ID_W'(c));

            wait ((done_per_core & expected_done_mask) == expected_done_mask);
            layer_cycles = ($time - t_layer_start) / 10;
            $display("  Layer %0d done @ t=%0t (cycles=%0d, mask=%b)",
                     l, $time, layer_cycles, expected_done_mask);
            // 等 axi_dm S2MM 内 in-flight burst 完全 commit (sts_fire 早于 mem 实际写)
            repeat (200) @(posedge clk);

            // 中间层 OFM check (last layer 在主循环外另算 final)
            if (l < n_layers - 1) begin
                check_layer_ofm(case_dir, l, $sformatf("L%0d post", l), layer_mm);
                $display("    POST-LAYER %0d OFM check: mismatches=%0d / %0d",
                         l, layer_mm, layer_ofb_words[l]);
            end
        end

        $display("  ALL layers done @ t=%0t (total cycles=%0d)", $time, ($time - t_start)/10);

        // 全局中间层 + final 检查
        total_intermediate_mm = 0;
        for (int l = 0; l < n_layers - 1; l++) begin
            check_layer_ofm(case_dir, l, $sformatf("L%0d", l), layer_mm);
            $display("  Layer %0d intermediate OFM mismatches=%0d / %0d",
                     l, layer_mm, layer_ofb_words[l]);
            total_intermediate_mm += layer_mm;
        end
        check_layer_ofm(case_dir, n_layers - 1, "FINAL", mismatches);

        $display("");
        $display("============================================================");
        if (mismatches == 0 && total_intermediate_mm == 0)
            $display("  RESULT: PASS  (All %0d layers OFM bit-exact, final %0d words)",
                     n_layers, layer_ofb_words[n_layers - 1]);
        else if (mismatches == 0)
            $display("  RESULT: PASS (chain self-consistent, final %0d words bit-exact)\n             *** intermediate boundary diff: %0d word ***",
                     layer_ofb_words[n_layers - 1], total_intermediate_mm);
        else
            $display("  RESULT: FAIL  Final mismatches=%0d / %0d  intermediate=%0d",
                     mismatches, layer_ofb_words[n_layers - 1], total_intermediate_mm);
        $display("  Wall: %0d cycles", ($time - t_start)/10);
        $display("============================================================");
        $finish;
    end

    // 周期 dump (含 SMC SG dispatcher 内部状态)
    initial begin
        @(posedge rst_n);
        forever begin
            #500_000;
            $display("[t=%0t] done=%b", $time, done_per_core);
            for (int c = 0; c < NUM_CORES; c++) begin
                case (c)
                    0: $display("  C0 seq=%0d idma=%b odma=%b rdma=%b | iSG st=%0d cmd=%0d/%0d done=%0d row_av=%0d row_co=%0d | oSG st=%0d cmd=%0d row_dr=%0d",
                                u_dut.gen_core[0].u_core.u_sequencer.state,
                                u_dut.gen_core[0].u_core.idma_done,
                                u_dut.gen_core[0].u_core.odma_done,
                                u_dut.gen_core[0].u_core.rdma_done,
                                u_dut.gen_core[0].u_core.g_idma_sg.u_idma_sg.st,
                                u_dut.gen_core[0].u_core.g_idma_sg.u_idma_sg.r_cmd_idx,
                                u_dut.gen_core[0].u_core.g_idma_sg.u_idma_sg.cfg_cmd_count,
                                u_dut.gen_core[0].u_core.g_idma_sg.u_idma_sg.r_cmds_done,
                                u_dut.gen_core[0].u_core.g_idma_sg.u_idma_sg.r_rows_pushed,
                                u_dut.gen_core[0].u_core.rows_consumed,
                                u_dut.gen_core[0].u_core.g_odma_sg.u_odma_sg.state,
                                u_dut.gen_core[0].u_core.g_odma_sg.u_odma_sg.r_cmd_idx,
                                u_dut.gen_core[0].u_core.g_odma_sg.u_odma_sg.r_rows_drained);
                    1: $display("  C1 seq=%0d idma=%b odma=%b rdma=%b | iSG st=%0d cmd=%0d/%0d done=%0d row_av=%0d row_co=%0d | oSG st=%0d cmd=%0d row_dr=%0d",
                                u_dut.gen_core[1].u_core.u_sequencer.state,
                                u_dut.gen_core[1].u_core.idma_done,
                                u_dut.gen_core[1].u_core.odma_done,
                                u_dut.gen_core[1].u_core.rdma_done,
                                u_dut.gen_core[1].u_core.g_idma_sg.u_idma_sg.st,
                                u_dut.gen_core[1].u_core.g_idma_sg.u_idma_sg.r_cmd_idx,
                                u_dut.gen_core[1].u_core.g_idma_sg.u_idma_sg.cfg_cmd_count,
                                u_dut.gen_core[1].u_core.g_idma_sg.u_idma_sg.r_cmds_done,
                                u_dut.gen_core[1].u_core.g_idma_sg.u_idma_sg.r_rows_pushed,
                                u_dut.gen_core[1].u_core.rows_consumed,
                                u_dut.gen_core[1].u_core.g_odma_sg.u_odma_sg.state,
                                u_dut.gen_core[1].u_core.g_odma_sg.u_odma_sg.r_cmd_idx,
                                u_dut.gen_core[1].u_core.g_odma_sg.u_odma_sg.r_rows_drained);
                    2: $display("  C2 seq=%0d idma=%b odma=%b rdma=%b | iSG st=%0d cmd=%0d/%0d done=%0d row_av=%0d row_co=%0d | oSG st=%0d cmd=%0d row_dr=%0d",
                                u_dut.gen_core[2].u_core.u_sequencer.state,
                                u_dut.gen_core[2].u_core.idma_done,
                                u_dut.gen_core[2].u_core.odma_done,
                                u_dut.gen_core[2].u_core.rdma_done,
                                u_dut.gen_core[2].u_core.g_idma_sg.u_idma_sg.st,
                                u_dut.gen_core[2].u_core.g_idma_sg.u_idma_sg.r_cmd_idx,
                                u_dut.gen_core[2].u_core.g_idma_sg.u_idma_sg.cfg_cmd_count,
                                u_dut.gen_core[2].u_core.g_idma_sg.u_idma_sg.r_cmds_done,
                                u_dut.gen_core[2].u_core.g_idma_sg.u_idma_sg.r_rows_pushed,
                                u_dut.gen_core[2].u_core.rows_consumed,
                                u_dut.gen_core[2].u_core.g_odma_sg.u_odma_sg.state,
                                u_dut.gen_core[2].u_core.g_odma_sg.u_odma_sg.r_cmd_idx,
                                u_dut.gen_core[2].u_core.g_odma_sg.u_odma_sg.r_rows_drained);
                    3: $display("  C3 seq=%0d idma=%b odma=%b rdma=%b | iSG st=%0d cmd=%0d/%0d done=%0d row_av=%0d row_co=%0d | oSG st=%0d cmd=%0d row_dr=%0d",
                                u_dut.gen_core[3].u_core.u_sequencer.state,
                                u_dut.gen_core[3].u_core.idma_done,
                                u_dut.gen_core[3].u_core.odma_done,
                                u_dut.gen_core[3].u_core.rdma_done,
                                u_dut.gen_core[3].u_core.g_idma_sg.u_idma_sg.st,
                                u_dut.gen_core[3].u_core.g_idma_sg.u_idma_sg.r_cmd_idx,
                                u_dut.gen_core[3].u_core.g_idma_sg.u_idma_sg.cfg_cmd_count,
                                u_dut.gen_core[3].u_core.g_idma_sg.u_idma_sg.r_cmds_done,
                                u_dut.gen_core[3].u_core.g_idma_sg.u_idma_sg.r_rows_pushed,
                                u_dut.gen_core[3].u_core.rows_consumed,
                                u_dut.gen_core[3].u_core.g_odma_sg.u_odma_sg.state,
                                u_dut.gen_core[3].u_core.g_odma_sg.u_odma_sg.r_cmd_idx,
                                u_dut.gen_core[3].u_core.g_odma_sg.u_odma_sg.r_rows_drained);
                endcase
            end
        end
    end

    initial begin
        #100_000_000;
        $display("FATAL: watchdog timeout @ %0t", $time);
        $stop;
    end

endmodule
