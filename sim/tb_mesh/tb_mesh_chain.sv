`timescale 1ns/1ps
`include "flux_cnn_params.svh"

// =============================================================================
// tb_mesh_chain.sv  --  Mesh 多层 chain TB (P0: single-core mesh chain)
//
// 跑 toolchain/run_multicore_chain.py --mesh 生成的 case 目录:
//   cases/<name>/multicore_meta.txt
//   cases/<name>/chain_data/layer<N>/  (每层 ifb/wb/expected_ofm/rdma/config)
//   cases/<name>/core<i>/layer<L>_desc_list.hex   (per-(core, layer) desc)
//   cases/<name>/mem<i>/mem<i>_layer<L>_cmds.txt  (per-(mem, layer) cmd list)
//
// Phase 6 P0 限制: 仅支持 single-core (n_cores=1), n_split=1. Step F.2 加 W slice.
//
// 工作流 (per layer):
//   1. host 配 ConvCore[0]: IFB_STRIP_ROWS / IFB_RING_WORDS (mesh ring) +
//      OFM_TDEST / OFM_OPCODE (出 OFM 写 mem[0].ddr_mem)
//   2. host 写 DESC_LIST_BASE → start_dfe → wait dfe_done
//   3. host 写 start_layer (CTRL[5])
//   4. fork:
//      a. TB 解析 mem0_layer{L}_cmds.txt → 一条条写 mem AXI-Lite (5 寄存器 +
//         TRIGGER) → poll STATUS[1]=done sticky
//      b. host 等 done_per_core[0] 拉高
//   5. 下一层
//
// 所有层完成后比对 mem[0].ddr_mem 内最后一层 OFM 区 vs 最后一层 expected_ofm.txt
// =============================================================================

module tb_mesh_chain;
    timeunit 1ns; timeprecision 1ps;

    // ---- 参数 (固定 4-core mesh, 但 P0 仅 ConvCore[0]/Mem[0] 干活) ----
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
    localparam int OFB_WIDTH   = NUM_COL * DATA_WIDTH;       // 128

    // cfg_regs 关键 addr
    localparam [11:0] ADDR_CTRL             = `FLUX_ADDR_CTRL;
    localparam [11:0] ADDR_DESC_LIST_BASE   = `FLUX_ADDR_DESC_LIST_BASE;
    localparam [11:0] ADDR_DESC_COUNT       = `FLUX_ADDR_DESC_COUNT;
    localparam [11:0] ADDR_SKIP_IDMA        = `FLUX_ADDR_SKIP_IDMA;
    localparam [11:0] ADDR_IFB_STRIP_ROWS   = `FLUX_ADDR_IFB_STRIP_ROWS;
    localparam [11:0] ADDR_IFB_RING_WORDS   = `FLUX_ADDR_IFB_RING_WORDS;
    localparam [11:0] ADDR_OFM_TDEST        = `FLUX_ADDR_OFM_TDEST;
    localparam [11:0] ADDR_OFM_OPCODE       = `FLUX_ADDR_OFM_OPCODE;

    logic clk = 0;  always #5 clk = ~clk;
    logic rst_n = 0;

    // ---- DUT 信号 ----
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

    int errors = 0;
    // mesh fire counter (debug)
    int mesh_in_fire [0:7];   // mem→mesh 各 LOCAL idx 的 fire 计数 (即 mem/conv 发包 count)
    int mesh_out_fire[0:7];   // mesh→conv/mem 各 LOCAL idx 的 fire 计数 (即 mem/conv 收包 count)
    initial for (int i = 0; i < 8; i++) begin mesh_in_fire[i]=0; mesh_out_fire[i]=0; end
    always @(posedge clk) begin
        if (rst_n) begin
            for (int i = 0; i < 8; i++) begin
                if (u_dut.u_mesh.s_axis_local_tvalid[i] && u_dut.u_mesh.s_axis_local_tready[i])
                    mesh_in_fire[i] <= mesh_in_fire[i] + 1;
                if (u_dut.u_mesh.m_axis_local_tvalid[i] && u_dut.u_mesh.m_axis_local_tready[i])
                    mesh_out_fire[i] <= mesh_out_fire[i] + 1;
            end
        end
    end

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

    // ---- DDR mock (WB / RDMA / desc 走这个, ConvCore.bus_ar/r 拉) ----
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
        .DEPTH(1048576*8)         // 128 MB DDR (装 desc/WB/RDMA)
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

    // ---- multicore_meta.txt 解析 ----
    int      n_layers;
    longint  layer_ddr_ifb [0:31];     // 每层 IFB 在 mem[c].ddr_mem 内 byte offset (broadcast 到 4 个 mem 同位置)
    longint  layer_ddr_wb  [0:31];
    longint  layer_ddr_ofb [0:31];
    longint  layer_ddr_rdma[0:31];
    int      layer_ifb_words [0:31];
    int      layer_wb_words  [0:31];
    int      layer_ofb_words [0:31];
    int      layer_rdma_words[0:31];
    int      layer_preload_ifb[0:31];
    int      layer_h_out     [0:31];        // 逐层 OFM 比对用
    int      layer_w_out     [0:31];
    int      layer_c_out     [0:31];
    string   layer_dirs    [0:31];

    // 多核 (NUM_CORES = 4): 每 (core, layer) 独立 desc base/count, 每 (mem, layer) 独立 cmd file/count
    longint  core_layer_desc_base [0:NUM_CORES-1][0:31];
    int      core_layer_desc_count[0:NUM_CORES-1][0:31];
    int      mem_layer_cmd_count  [0:NUM_CORES-1][0:31];
    string   mem_layer_cmd_files  [0:NUM_CORES-1][0:31];

    // mesh-specific meta (driver 写在 ; ---- mesh-specific 段)
    int      mesh_n_cores = 1;
    int      mem_layer_w_out_start[0:NUM_CORES-1][0:31];
    int      mem_layer_my_w_out   [0:NUM_CORES-1][0:31];
    int      final_h_out, final_w_out, final_c_out;

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
        for (int l = 0; l < 32; l++) begin
            for (int c = 0; c < NUM_CORES; c++) begin
                core_layer_desc_count[c][l] = 0;
                mem_layer_cmd_count[c][l]   = 0;
            end
            layer_preload_ifb[l] = 0;
        end

        while (!$feof(fd)) begin
            void'($fgets(line, fd));
            if (line.len() == 0) continue;
            if (line.getc(0) == ";") continue;
            parsed_num = 1'b0;
            if      ($sscanf(line, "%s = 0x%h", key, val) == 2) parsed_num = 1'b1;
            else if ($sscanf(line, "%s = %d",   key, val) == 2) parsed_num = 1'b1;
            else if ($sscanf(line, "%s = %s",   key, val_s) == 2) begin
                // string 字段
                if (key.len() > 6 && key.substr(0,5) == "LAYER_") begin
                    p1 = -1;
                    for (int i = 6; i < key.len(); i++)
                        if (key.getc(i) == "_") begin p1 = i; break; end
                    if (p1 > 0) begin
                        layer_id = key.substr(6, p1-1).atoi();
                        if (key.substr(p1+1, key.len()-1) == "DIR")
                            layer_dirs[layer_id] = val_s;
                    end
                end else if (key.len() > 4 && key.substr(0,3) == "MEM_") begin
                    // MEM_<m>_LAYER_<l>_CMD_FILE
                    p1 = -1; p2 = -1;
                    for (int i = 4; i < key.len(); i++)
                        if (key.getc(i) == "_") begin p1 = i; break; end
                    if (p1 > 0 && p1 + 6 < key.len() &&
                        key.substr(p1+1, p1+5) == "LAYER" && key.getc(p1+6) == "_") begin
                        for (int j = p1+7; j < key.len(); j++)
                            if (key.getc(j) == "_") begin p2 = j; break; end
                        if (p2 > 0) begin
                            core_id  = key.substr(4, p1-1).atoi();
                            layer_id = key.substr(p1+7, p2-1).atoi();
                            suffix   = key.substr(p2+1, key.len()-1);
                            if (core_id < NUM_CORES && suffix == "CMD_FILE")
                                mem_layer_cmd_files[core_id][layer_id] = val_s;
                        end
                    end
                end
                continue;
            end else continue;
            if (!parsed_num) continue;

            // 数字键
            if      (key == "NUM_LAYERS")   n_layers     = val;
            else if (key == "MESH_N_CORES") mesh_n_cores = val;
            else if (key == "FINAL_H_OUT")  final_h_out  = val;
            else if (key == "FINAL_W_OUT")  final_w_out  = val;
            else if (key == "FINAL_C_OUT")  final_c_out  = val;
            else if (key.len() > 4 && key.substr(0,3) == "MEM_") begin
                // MEM_<m>_LAYER_<l>_{CMD_COUNT, W_OUT_START, MY_W_OUT}
                p1 = -1; p2 = -1;
                for (int i = 4; i < key.len(); i++)
                    if (key.getc(i) == "_") begin p1 = i; break; end
                if (p1 > 0 && p1 + 6 < key.len() &&
                    key.substr(p1+1, p1+5) == "LAYER" && key.getc(p1+6) == "_") begin
                    for (int j = p1+7; j < key.len(); j++)
                        if (key.getc(j) == "_") begin p2 = j; break; end
                    if (p2 > 0) begin
                        core_id  = key.substr(4, p1-1).atoi();
                        layer_id = key.substr(p1+7, p2-1).atoi();
                        suffix   = key.substr(p2+1, key.len()-1);
                        if (core_id < NUM_CORES) begin
                            if      (suffix == "CMD_COUNT")   mem_layer_cmd_count   [core_id][layer_id] = val;
                            else if (suffix == "W_OUT_START") mem_layer_w_out_start [core_id][layer_id] = val;
                            else if (suffix == "MY_W_OUT")    mem_layer_my_w_out    [core_id][layer_id] = val;
                        end
                    end
                end
            end else if (key.len() > 5 && key.substr(0,4) == "CORE_") begin
                // CORE_0_LAYER_<l>_DESC_BASE / DESC_COUNT
                p1 = -1; p2 = -1;
                for (int i = 5; i < key.len(); i++)
                    if (key.getc(i) == "_") begin p1 = i; break; end
                if (p1 > 0 && p1 + 6 < key.len() &&
                    key.substr(p1+1, p1+5) == "LAYER" && key.getc(p1+6) == "_") begin
                    for (int j = p1+7; j < key.len(); j++)
                        if (key.getc(j) == "_") begin p2 = j; break; end
                    if (p2 > 0) begin
                        core_id  = key.substr(5, p1-1).atoi();
                        layer_id = key.substr(p1+7, p2-1).atoi();
                        suffix   = key.substr(p2+1, key.len()-1);
                        if (core_id < NUM_CORES) begin
                            if      (suffix == "DESC_BASE")  core_layer_desc_base [core_id][layer_id] = val;
                            else if (suffix == "DESC_COUNT") core_layer_desc_count[core_id][layer_id] = val;
                        end
                    end
                end
            end else if (key.len() > 6 && key.substr(0,5) == "LAYER_") begin
                p = -1;
                for (int i = 6; i < key.len(); i++)
                    if (key.getc(i) == "_") begin p = i; break; end
                if (p > 0) begin
                    layer_id = key.substr(6, p-1).atoi();
                    suffix = key.substr(p+1, key.len()-1);
                    case (suffix)
                        "IFB_WORDS"  : layer_ifb_words[layer_id] = val;
                        "WB_WORDS"   : layer_wb_words[layer_id]  = val;
                        "OFB_WORDS"  : layer_ofb_words[layer_id] = val;
                        "RDMA_WORDS" : layer_rdma_words[layer_id]= val;
                        "PRELOAD_IFB": layer_preload_ifb[layer_id] = val;
                        "H_OUT"      : layer_h_out[layer_id]     = val;
                        "W_OUT"      : layer_w_out[layer_id]     = val;
                        "C_OUT"      : layer_c_out[layer_id]     = val;
                        "DDR_IFB"    : layer_ddr_ifb [layer_id] = val;
                        "DDR_WB"     : layer_ddr_wb  [layer_id] = val;
                        "DDR_OFB"    : layer_ddr_ofb [layer_id] = val;
                        "DDR_RDMA"   : layer_ddr_rdma[layer_id] = val;
                        default      : ;
                    endcase
                end
            end
        end
        $fclose(fd);
    endtask

    // ---- preload buffers ----
    logic [BUS_DATA_W-1:0]                ifb_arr  [0:524287];
    logic [NUM_COL*NUM_PE*DATA_WIDTH-1:0] wb_arr   [0:4095];
    logic [BUS_DATA_W-1:0]                rdma_arr [0:65535];
    logic [BUS_DATA_W-1:0]                desc_arr [0:8191];
    logic [OFB_WIDTH-1:0]                 exp_arr  [0:524287];

    // ---- preload helpers ----
    // mesh 模式: IFB broadcast 到 0..mesh_n_cores-1 个 mem (相同位置, 模拟多 DDR / 单 DDR + 路由)
    task automatic preload_layer_ifb_to_mem(input string layer_dir, input longint base, input int n_words,
                                              input int n_active);
        int base_w = base / 16;
        $readmemh($sformatf("%s/ifb.txt", layer_dir), ifb_arr);
        for (int c = 0; c < n_active; c++) begin
            case (c)
                0: for (int i = 0; i < n_words; i++) u_dut.gen_mem[0].u_mem.ddr_mem[base_w + i] = ifb_arr[i];
                1: for (int i = 0; i < n_words; i++) u_dut.gen_mem[1].u_mem.ddr_mem[base_w + i] = ifb_arr[i];
                2: for (int i = 0; i < n_words; i++) u_dut.gen_mem[2].u_mem.ddr_mem[base_w + i] = ifb_arr[i];
                3: for (int i = 0; i < n_words; i++) u_dut.gen_mem[3].u_mem.ddr_mem[base_w + i] = ifb_arr[i];
                default:;
            endcase
        end
    endtask

    task automatic preload_layer_wb_to_ddr(input string layer_dir, input longint base, input int n_words);
        int base_w = base / 16;
        $readmemh($sformatf("%s/wb.txt", layer_dir), wb_arr);
        for (int i = 0; i < n_words; i++)
            for (int b = 0; b < 16; b++)
                u_ddr.mem[base_w + i*16 + b] = wb_arr[i][b*BUS_DATA_W +: BUS_DATA_W];
    endtask

    task automatic preload_layer_rdma_to_ddr(input string layer_dir, input longint base, input int n_words);
        int base_w = base / 16;
        int fd = $fopen($sformatf("%s/rdma_data.txt", layer_dir), "r");
        if (fd == 0) return;
        $fclose(fd);
        for (int i = 0; i < 65536; i++) rdma_arr[i] = '0;
        $readmemh($sformatf("%s/rdma_data.txt", layer_dir), rdma_arr);
        for (int i = 0; i < n_words; i++) u_ddr.mem[base_w + i] = rdma_arr[i];
    endtask

    task automatic preload_layer_desc_to_ddr(input string case_dir, input int core_id,
                                                input int layer_idx, input longint base);
        string path, l2;
        int    base_w = base / 16;
        if (layer_idx < 10) l2 = $sformatf("0%0d", layer_idx);
        else                l2 = $sformatf("%0d",  layer_idx);
        path = $sformatf("%s/core%0d/layer%s_desc_list.hex", case_dir, core_id, l2);
        $readmemh(path, desc_arr);
        for (int i = 0; i < 1024; i++) u_ddr.mem[base_w + i] = desc_arr[i];
    endtask

    // ---- AXI-Lite write/read tasks (ConvCore CSR) ----
    task automatic axi_lite_write(input int core_id, input [11:0] reg_addr, input [31:0] data);
        @(posedge clk);
        csr_awaddr  <= {2'(core_id), reg_addr};
        csr_awvalid <= 1'b1;
        csr_wdata   <= data; csr_wstrb <= '1; csr_wvalid <= 1'b1;
        do @(posedge clk); while (!(csr_awvalid && csr_awready));
        csr_awvalid <= 1'b0;
        while (!(csr_wvalid && csr_wready)) @(posedge clk);
        csr_wvalid <= 1'b0;
        csr_bready <= 1'b1;
        do @(posedge clk); while (!(csr_bvalid && csr_bready));
        csr_bready <= 1'b0;
    endtask

    // host mem CSR 总线只有一条, 4 个 mem fork 并发 write 必须互斥
    semaphore mem_csr_sem = new(1);

    // mem CSR write/read
    task automatic mem_axi_lite_write(input int mem_id, input [11:0] reg_addr, input [31:0] data);
        mem_csr_sem.get(1);
        @(posedge clk);
        mem_csr_awaddr  <= {2'(mem_id), reg_addr};
        mem_csr_awvalid <= 1'b1;
        mem_csr_wdata   <= data; mem_csr_wstrb <= '1; mem_csr_wvalid <= 1'b1;
        do @(posedge clk); while (!(mem_csr_awvalid && mem_csr_awready));
        mem_csr_awvalid <= 1'b0;
        while (!(mem_csr_wvalid && mem_csr_wready)) @(posedge clk);
        mem_csr_wvalid <= 1'b0;
        mem_csr_bready <= 1'b1;
        do @(posedge clk); while (!(mem_csr_bvalid && mem_csr_bready));
        mem_csr_bready <= 1'b0;
        mem_csr_sem.put(1);
    endtask

    task automatic mem_axi_lite_read(input int mem_id, input [11:0] reg_addr, output [31:0] data);
        mem_csr_sem.get(1);
        @(posedge clk);
        mem_csr_araddr  <= {2'(mem_id), reg_addr};
        mem_csr_arvalid <= 1'b1;
        do @(posedge clk); while (!(mem_csr_arvalid && mem_csr_arready));
        mem_csr_arvalid <= 1'b0;
        do @(posedge clk); while (!(mem_csr_rvalid));
        data = mem_csr_rdata;
        mem_csr_sem.put(1);
    endtask

    // ---- mem desc list preload + engine 启动 (Step F.4 优化路径) ----
    //   1. 把 desc list .hex 文件 preload 到 mem.ddr_mem 的某区域 (TB 选 8MB 处)
    //   2. host 写 mem CSR: DESC_LIST_ADDR / DESC_COUNT / CTRL[0]=1 启动
    //   3. mem 内 desc engine 自动消费 desc 顺序 trigger packet, 4 mem 真并行

    // mem desc list preload 区 (8 MB byte = word 0x80000)
    localparam int MEM_DESC_BASE_WORD = 32'h0008_0000;

    // desc data buffer (1 desc = 16 byte = 1 word)
    logic [BUS_DATA_W-1:0] mem_desc_arr [0:8191];

    task automatic preload_mem_desc_list(input int mem_id, input string case_dir, input string rel_file);
        string full_path;
        full_path = $sformatf("%s/%s", case_dir, rel_file);
        // 清缓冲, 防上次残留
        for (int i = 0; i < 8192; i++) mem_desc_arr[i] = '0;
        $readmemh(full_path, mem_desc_arr);
        case (mem_id)
            0: for (int i = 0; i < 8192; i++) u_dut.gen_mem[0].u_mem.ddr_mem[MEM_DESC_BASE_WORD + i] = mem_desc_arr[i];
            1: for (int i = 0; i < 8192; i++) u_dut.gen_mem[1].u_mem.ddr_mem[MEM_DESC_BASE_WORD + i] = mem_desc_arr[i];
            2: for (int i = 0; i < 8192; i++) u_dut.gen_mem[2].u_mem.ddr_mem[MEM_DESC_BASE_WORD + i] = mem_desc_arr[i];
            3: for (int i = 0; i < 8192; i++) u_dut.gen_mem[3].u_mem.ddr_mem[MEM_DESC_BASE_WORD + i] = mem_desc_arr[i];
            default:;
        endcase
    endtask

    // 启动 mem engine: 写 DESC_LIST_ADDR / DESC_COUNT / CTRL=1 (3 个 CSR write)
    task automatic mem_engine_start(input int mem_id, input int desc_count);
        mem_axi_lite_write(mem_id, 12'h000, MEM_DESC_BASE_WORD);  // DESC_LIST_ADDR
        mem_axi_lite_write(mem_id, 12'h004, desc_count);           // DESC_COUNT
        mem_axi_lite_write(mem_id, 12'h008, 32'd1);                // CTRL[0]=1 → start_pulse
    endtask

    // 等 mem engine done sticky (poll STATUS[1])
    task automatic mem_engine_wait_done(input int mem_id);
        logic [31:0] status;
        int          poll_cnt;
        poll_cnt = 0;
        do begin
            mem_axi_lite_read(mem_id, 12'h00C, status);
            poll_cnt++;
            if (poll_cnt > 200000) begin
                $display("FATAL: mem[%0d] engine done sticky timeout", mem_id);
                $stop;
            end
        end while (!status[1]);
    endtask

    // ---- 逐层 OFM 验证 task (修复 trivial PASS bug: 不再只比对最后一层) ----
    task automatic check_layer_ofm(input int L);
        automatic int cout_slices = (layer_c_out[L] + 15) / 16;
        automatic int W_OUT_FULL  = layer_w_out[L];
        automatic int H_OUT       = layer_h_out[L];
        automatic int ofb_base_w  = layer_ddr_ofb[L] / 16;
        automatic int layer_mismatch = 0;
        $readmemh($sformatf("%s/expected_ofm.txt", layer_dirs[L]), exp_arr);
        $display("\n[Layer %0d OFM check] H=%0d W=%0d cout_slices=%0d ofb_base_w=0x%08x",
                 L, H_OUT, W_OUT_FULL, cout_slices, ofb_base_w);
        for (int c = 0; c < mesh_n_cores; c++) begin
            automatic int w_start    = mem_layer_w_out_start[c][L];
            automatic int my_w       = mem_layer_my_w_out   [c][L];
            automatic int c_mismatch = 0;
            if (my_w == 0) continue;
            for (int r = 0; r < H_OUT; r++) begin
                for (int j = 0; j < my_w; j++) begin
                    for (int s = 0; s < cout_slices; s++) begin
                        automatic int src_w = ofb_base_w + r * W_OUT_FULL * cout_slices + (w_start + j) * cout_slices + s;
                        automatic int exp_w = r * W_OUT_FULL * cout_slices + (w_start + j) * cout_slices + s;
                        logic [BUS_DATA_W-1:0] got;
                        logic [OFB_WIDTH-1:0]  exp;
                        case (c)
                            0: got = u_dut.gen_mem[0].u_mem.ddr_mem[src_w];
                            1: got = u_dut.gen_mem[1].u_mem.ddr_mem[src_w];
                            2: got = u_dut.gen_mem[2].u_mem.ddr_mem[src_w];
                            3: got = u_dut.gen_mem[3].u_mem.ddr_mem[src_w];
                            default: got = 'x;
                        endcase
                        exp = exp_arr[exp_w];
                        if (got[OFB_WIDTH-1:0] !== exp) begin
                            if (layer_mismatch < 3)
                                $display("  L%0d FAIL c%0d r%0d w%0d s%0d got=%h exp=%h",
                                         L, c, r, w_start+j, s, got[OFB_WIDTH-1:0], exp);
                            c_mismatch++;
                            layer_mismatch++;
                        end
                    end
                end
            end
            $display("  L%0d core %0d (w[%0d:%0d]): %s",
                     L, c, w_start, w_start+my_w,
                     c_mismatch == 0 ? "PASS" : $sformatf("FAIL %0d", c_mismatch));
        end
        if (layer_mismatch == 0)
            $display("  L%0d OFM bit-exact PASS", L);
        else
            errors += layer_mismatch;
    endtask

    // ---- 主流程 ----
    initial begin : main
        string case_dir;
        int    final_ofb_words;
        int    final_ofm_w_base;
        int    mismatch;

        // case_dir 通过 plusargs 传入
        if (!$value$plusargs("CASE_DIR=%s", case_dir)) begin
            $display("FATAL: pass +CASE_DIR=<path> as plusarg"); $stop;
        end

        csr_awaddr = 0; csr_awvalid = 0; csr_wdata = 0; csr_wstrb = 0;
        csr_wvalid = 0; csr_bready = 0;
        mem_csr_awaddr = 0; mem_csr_awvalid = 0;
        mem_csr_wdata = 0; mem_csr_wstrb = 0; mem_csr_wvalid = 0; mem_csr_bready = 0;

        #20 rst_n = 1;
        #10;

        $display("==============================================================");
        $display("== tb_mesh_chain: %s ==", case_dir);
        $display("==============================================================");

        parse_meta(case_dir);
        $display("  n_layers = %0d  mesh_n_cores = %0d  final=%0dx%0dx%0d",
                 n_layers, mesh_n_cores, final_h_out, final_w_out, final_c_out);
        for (int l = 0; l < n_layers; l++) begin
            $display("  layer %0d: dir=%s ifb=%0d wb=%0d ofb=%0d rdma=%0d preload_ifb=%0d ddr_ifb=0x%08x ddr_ofb=0x%08x",
                     l, layer_dirs[l], layer_ifb_words[l], layer_wb_words[l],
                     layer_ofb_words[l], layer_rdma_words[l], layer_preload_ifb[l],
                     layer_ddr_ifb[l], layer_ddr_ofb[l]);
            for (int c = 0; c < mesh_n_cores; c++)
                $display("    core %0d: desc_count=%0d cmd_count=%0d w_out_start=%0d my_w_out=%0d",
                         c, core_layer_desc_count[c][l], mem_layer_cmd_count[c][l],
                         mem_layer_w_out_start[c][l], mem_layer_my_w_out[c][l]);
        end

        // ---- preload (IFB 到 mem; WB/RDMA/ConvCore desc 在 axi_slave_mem; mem desc list 进 mem.ddr_mem) ----
        for (int l = 0; l < n_layers; l++) begin
            if (l == 0 || layer_preload_ifb[l] == 1)
                preload_layer_ifb_to_mem(layer_dirs[l], layer_ddr_ifb[l], layer_ifb_words[l],
                                          mesh_n_cores);
            preload_layer_wb_to_ddr(layer_dirs[l], layer_ddr_wb[l], layer_wb_words[l]);
            if (layer_rdma_words[l] > 0)
                preload_layer_rdma_to_ddr(layer_dirs[l], layer_ddr_rdma[l], layer_rdma_words[l]);
            // 每核 ConvCore desc 各自 preload (axi_slave_mem)
            for (int c = 0; c < mesh_n_cores; c++)
                if (core_layer_desc_count[c][l] > 0)
                    preload_layer_desc_to_ddr(case_dir, c, l, core_layer_desc_base[c][l]);
        end
        $display("  preload data + ConvCore desc done");

        // ---- per-layer 启动序列 (per-core 并行) ----
        for (int L = 0; L < n_layers; L++) begin
            $display("\n[Layer %0d] start (mesh_n_cores=%0d)", L, mesh_n_cores);

            // 1. host 配每核 mesh 模式 + DESC + start_dfe
            //   (IFB_STRIP_ROWS / IFB_RING_WORDS 不写, desc CFG_WRITE 接管)
            for (int c = 0; c < mesh_n_cores; c++) begin
                if (core_layer_desc_count[c][L] == 0) continue;
                axi_lite_write(c, ADDR_SKIP_IDMA,      32'd1);
                axi_lite_write(c, ADDR_OFM_TDEST,      {24'd0, 4'd0, c[3:0]});  // mem[c] @ (0, c)
                axi_lite_write(c, ADDR_OFM_OPCODE,     32'h0000_0005);          // WRITE_DDR_OFB
                axi_lite_write(c, ADDR_DESC_LIST_BASE, core_layer_desc_base [c][L]);
                axi_lite_write(c, ADDR_DESC_COUNT,     core_layer_desc_count[c][L]);
                axi_lite_write(c, ADDR_CTRL,           32'h0000_0010);          // start_dfe
            end

            // 2. 等所有 active 核 dfe_busy 0→1→0 (静态 unroll, NUM_CORES=4)
            if (mesh_n_cores >= 1 && core_layer_desc_count[0][L] > 0) begin
                wait (u_dut.gen_core[0].u_conv.u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[0].u_conv.u_core.dfe_busy == 1'b0);
            end
            if (mesh_n_cores >= 2 && core_layer_desc_count[1][L] > 0) begin
                wait (u_dut.gen_core[1].u_conv.u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[1].u_conv.u_core.dfe_busy == 1'b0);
            end
            if (mesh_n_cores >= 3 && core_layer_desc_count[2][L] > 0) begin
                wait (u_dut.gen_core[2].u_conv.u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[2].u_conv.u_core.dfe_busy == 1'b0);
            end
            if (mesh_n_cores >= 4 && core_layer_desc_count[3][L] > 0) begin
                wait (u_dut.gen_core[3].u_conv.u_core.dfe_busy == 1'b1);
                wait (u_dut.gen_core[3].u_conv.u_core.dfe_busy == 1'b0);
            end

            // 3. preload mem desc list 到 mem.ddr_mem (Step F.4: mem 内置 desc engine)
            for (int c = 0; c < mesh_n_cores; c++) begin
                if (mem_layer_cmd_count[c][L] > 0)
                    preload_mem_desc_list(c, case_dir, mem_layer_cmd_files[c][L]);
            end

            // 4. 给所有核 start_layer + 启动各 mem desc engine (3 个 CSR write / mem)
            for (int c = 0; c < mesh_n_cores; c++) begin
                if (core_layer_desc_count[c][L] == 0) continue;
                axi_lite_write(c, ADDR_CTRL, 32'h0000_0020);
            end
            for (int c = 0; c < mesh_n_cores; c++) begin
                if (mem_layer_cmd_count[c][L] > 0)
                    mem_engine_start(c, mem_layer_cmd_count[c][L]);
            end

            // 5. fork: 4 mem engine done + 4 ConvCore done wait
            fork
                // mem engine done waits (静态 unroll)
                begin
                    if (mem_layer_cmd_count[0][L] > 0) mem_engine_wait_done(0);
                end
                begin
                    if (mem_layer_cmd_count[1][L] > 0) mem_engine_wait_done(1);
                end
                begin
                    if (mem_layer_cmd_count[2][L] > 0) mem_engine_wait_done(2);
                end
                begin
                    if (mem_layer_cmd_count[3][L] > 0) mem_engine_wait_done(3);
                end
                // ConvCore done waits
                begin
                    if (mesh_n_cores >= 1 && core_layer_desc_count[0][L] > 0) begin
                        wait (done_per_core[0] == 1'b1);
                        $display("  ConvCore[0] layer %0d done @ t=%0t", L, $time);
                    end
                end
                begin
                    if (mesh_n_cores >= 2 && core_layer_desc_count[1][L] > 0) begin
                        wait (done_per_core[1] == 1'b1);
                        $display("  ConvCore[1] layer %0d done @ t=%0t", L, $time);
                    end
                end
                begin
                    if (mesh_n_cores >= 3 && core_layer_desc_count[2][L] > 0) begin
                        wait (done_per_core[2] == 1'b1);
                        $display("  ConvCore[2] layer %0d done @ t=%0t", L, $time);
                    end
                end
                begin
                    if (mesh_n_cores >= 4 && core_layer_desc_count[3][L] > 0) begin
                        wait (done_per_core[3] == 1'b1);
                        $display("  ConvCore[3] layer %0d done @ t=%0t", L, $time);
                    end
                end
            join
            // 等若干拍让最后的 BVALID + sticky 信号稳
            repeat (50) @(posedge clk);

            // 每层结束后立即比对该层 OFM (修复 trivial PASS bug)
            check_layer_ofm(L);
        end

        // 逐层 OFM 验证已经覆盖所有 layer (含 last), errors 累计
        $display("\n==============================================================");
        if (errors == 0)
            $display("  RESULT: PASS  (mesh chain end-to-end, %0d layers, per-layer OFM bit-exact)", n_layers);
        else
            $display("  RESULT: FAIL  errors=%0d (across all layers)", errors);
        $display("==============================================================");
        $finish;
    end

    initial begin
        #500_000_000 $display("WATCHDOG TIMEOUT"); $finish;
    end
endmodule
