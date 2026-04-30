`timescale 1ns/1ps

// =============================================================================
// tb_idma_ctrl.sv  --  idma_ctrl + axi_dm 联合自测
//
// 目的: 验证 idma_ctrl 通过 axi_dm (Xilinx DataMover MM2S) 把 DDR 数据搬到 IFB
// SRAM, 内容 bit-exact.
//
// 测试场景: 一次性整图搬运 (无 ring wrap, strip_rows = h_total)
//   H_in    = 4 行
//   ky_step = 8 beats/row  (即 W_IN × cin_slices = 8, 每行 128 字节)
//   total   = 32 beats = 512 bytes
//   ring    = 32 words   (= 4 × 8)
//   src DDR: mem[i] = {84'd0, 12'h0, 32'(i)}  // 第 i 个 beat 低 32-bit = i
//
// 通过条件:
//   1. done == 1
//   2. err == 0
//   3. IFB[i] == DDR[i] for i = 0..31
// =============================================================================

module tb_idma_ctrl;

    localparam int ADDR_W      = 32;
    localparam int DATA_W      = 128;
    localparam int SRAM_ADDR_W = 6;       // 64-word IFB
    localparam int IFB_DEPTH   = 1 << SRAM_ADDR_W;
    localparam int DDR_DEPTH   = 64;

    localparam int H_IN        = 4;
    localparam int KY_STEP     = 8;       // beats per row
    localparam int TOTAL_BEATS = H_IN * KY_STEP;
    localparam int RING_WORDS  = TOTAL_BEATS;
    localparam int SRC_BASE    = 32'h0000_0000;

    logic clk = 1'b0;
    logic rst_n = 1'b0;
    initial forever #5 clk = ~clk;

    // ---- idma_ctrl <→ axi_dm (MM2S 内通道) ----
    logic        cmd_tvalid, cmd_tready;
    logic [71:0] cmd_tdata;
    logic        data_tvalid, data_tready;
    logic [127:0] data_tdata;
    logic [15:0] data_tkeep;
    logic        data_tlast;
    logic        sts_tvalid, sts_tready;
    logic [7:0]  sts_tdata;

    // ---- axi_dm <→ DDR (AXI4 master 接 axi_slave_mem) ----
    logic [3:0]   ar_id;
    logic [31:0]  ar_addr;
    logic [7:0]   ar_len;
    logic [2:0]   ar_size;
    logic [1:0]   ar_burst;
    logic         ar_valid, ar_ready;
    logic [127:0] r_data;
    logic [1:0]   r_resp;
    logic         r_last, r_valid, r_ready;

    // S2MM 通道全部 tie 0 (我们只测 MM2S)
    logic         s2mm_cmd_tvalid       = 1'b0;
    logic [71:0]  s2mm_cmd_tdata        = 72'b0;
    logic         s2mm_sts_tready_tie   = 1'b1;
    logic         s2mm_aw_ready_tie     = 1'b1;
    logic         s2mm_w_ready_tie      = 1'b1;
    logic         s2mm_b_valid_tie      = 1'b0;
    logic [127:0] s2mm_data_tdata_tie   = 128'b0;
    logic [15:0]  s2mm_data_tkeep_tie   = 16'b0;
    logic         s2mm_data_tlast_tie   = 1'b0;
    logic         s2mm_data_tvalid_tie  = 1'b0;

    // ---- IFB SRAM 写端 ----
    logic                  ifb_we;
    logic [SRAM_ADDR_W-1:0] ifb_waddr;
    logic [DATA_W-1:0]     ifb_wdata;

    // ---- idma_ctrl cfg/control ----
    logic        idma_start;
    logic        idma_done;
    logic        idma_busy;
    logic        idma_err;
    logic [15:0] rows_consumed;
    logic [15:0] rows_available;

    // =========================================================================
    // DUT: idma_ctrl
    // =========================================================================
    idma_ctrl #(
        .ADDR_W      (ADDR_W),
        .DATA_W      (DATA_W),
        .SRAM_ADDR_W (SRAM_ADDR_W)
    ) u_idma_ctrl (
        .clk                (clk),
        .rst_n              (rst_n),
        .start              (idma_start),
        .done               (idma_done),
        .busy               (idma_busy),
        .err                (idma_err),
        .src_base           (SRC_BASE),
        .byte_len           ('0),
        .cfg_h_in_total         (16'(H_IN)),
        .cfg_ifb_strip_rows     (8'(H_IN)),                 // 整图都装得下, ring 不 wrap
        .cfg_ifb_ky_step        (20'(KY_STEP)),
        .cfg_ifb_ring_words     (20'(RING_WORDS)),
        .cfg_ddr_ifm_row_stride (ADDR_W'(KY_STEP * (DATA_W/8))),  // single-tile = cmd_btt
        .rows_consumed          (rows_consumed),
        .rows_available         (rows_available),

        .mm2s_cmd_tvalid    (cmd_tvalid),
        .mm2s_cmd_tready    (cmd_tready),
        .mm2s_cmd_tdata     (cmd_tdata),
        .mm2s_data_tvalid   (data_tvalid),
        .mm2s_data_tready   (data_tready),
        .mm2s_data_tdata    (data_tdata),
        .mm2s_data_tkeep    (data_tkeep),
        .mm2s_data_tlast    (data_tlast),
        .mm2s_sts_tvalid    (sts_tvalid),
        .mm2s_sts_tready    (sts_tready),
        .mm2s_sts_tdata     (sts_tdata),

        .ifb_we             (ifb_we),
        .ifb_waddr          (ifb_waddr),
        .ifb_wdata          (ifb_wdata)
    );

    // =========================================================================
    // axi_dm IP (DataMover) — 只用 MM2S, S2MM 端口全 tie
    // =========================================================================
    axi_dm u_axi_dm (
        .m_axi_mm2s_aclk            (clk),
        .m_axi_mm2s_aresetn         (rst_n),
        .mm2s_err                   (),
        .m_axis_mm2s_cmdsts_aclk    (clk),
        .m_axis_mm2s_cmdsts_aresetn (rst_n),

        .s_axis_mm2s_cmd_tvalid     (cmd_tvalid),
        .s_axis_mm2s_cmd_tready     (cmd_tready),
        .s_axis_mm2s_cmd_tdata      (cmd_tdata),
        .m_axis_mm2s_sts_tvalid     (sts_tvalid),
        .m_axis_mm2s_sts_tready     (sts_tready),
        .m_axis_mm2s_sts_tdata      (sts_tdata),
        .m_axis_mm2s_sts_tkeep      (),
        .m_axis_mm2s_sts_tlast      (),

        .m_axi_mm2s_arid            (ar_id),
        .m_axi_mm2s_araddr          (ar_addr),
        .m_axi_mm2s_arlen           (ar_len),
        .m_axi_mm2s_arsize          (ar_size),
        .m_axi_mm2s_arburst         (ar_burst),
        .m_axi_mm2s_arprot          (),
        .m_axi_mm2s_arcache         (),
        .m_axi_mm2s_aruser          (),
        .m_axi_mm2s_arvalid         (ar_valid),
        .m_axi_mm2s_arready         (ar_ready),
        .m_axi_mm2s_rdata           (r_data),
        .m_axi_mm2s_rresp           (r_resp),
        .m_axi_mm2s_rlast           (r_last),
        .m_axi_mm2s_rvalid          (r_valid),
        .m_axi_mm2s_rready          (r_ready),
        .m_axis_mm2s_tdata          (data_tdata),
        .m_axis_mm2s_tkeep          (data_tkeep),
        .m_axis_mm2s_tlast          (data_tlast),
        .m_axis_mm2s_tvalid         (data_tvalid),
        .m_axis_mm2s_tready         (data_tready),

        // S2MM tie
        .m_axi_s2mm_aclk            (clk),
        .m_axi_s2mm_aresetn         (rst_n),
        .s2mm_err                   (),
        .m_axis_s2mm_cmdsts_awclk   (clk),
        .m_axis_s2mm_cmdsts_aresetn (rst_n),
        .s_axis_s2mm_cmd_tvalid     (s2mm_cmd_tvalid),
        .s_axis_s2mm_cmd_tready     (),
        .s_axis_s2mm_cmd_tdata      (s2mm_cmd_tdata),
        .m_axis_s2mm_sts_tvalid     (),
        .m_axis_s2mm_sts_tready     (s2mm_sts_tready_tie),
        .m_axis_s2mm_sts_tdata      (),
        .m_axis_s2mm_sts_tkeep      (),
        .m_axis_s2mm_sts_tlast      (),
        .m_axi_s2mm_awid            (),
        .m_axi_s2mm_awaddr          (),
        .m_axi_s2mm_awlen           (),
        .m_axi_s2mm_awsize          (),
        .m_axi_s2mm_awburst         (),
        .m_axi_s2mm_awprot          (),
        .m_axi_s2mm_awcache         (),
        .m_axi_s2mm_awuser          (),
        .m_axi_s2mm_awvalid         (),
        .m_axi_s2mm_awready         (s2mm_aw_ready_tie),
        .m_axi_s2mm_wdata           (),
        .m_axi_s2mm_wstrb           (),
        .m_axi_s2mm_wlast           (),
        .m_axi_s2mm_wvalid          (),
        .m_axi_s2mm_wready          (s2mm_w_ready_tie),
        .m_axi_s2mm_bresp           (2'b00),
        .m_axi_s2mm_bvalid          (s2mm_b_valid_tie),
        .m_axi_s2mm_bready          (),
        .s_axis_s2mm_tdata          (s2mm_data_tdata_tie),
        .s_axis_s2mm_tkeep          (s2mm_data_tkeep_tie),
        .s_axis_s2mm_tlast          (s2mm_data_tlast_tie),
        .s_axis_s2mm_tvalid         (s2mm_data_tvalid_tie),
        .s_axis_s2mm_tready         ()
    );

    // =========================================================================
    // DDR mock (axi_slave_mem) — 用作 axi_dm 的 read target
    //   只用读通道, 写通道 tie 0
    // =========================================================================
    axi_slave_mem #(
        .ADDR_W (ADDR_W),
        .DATA_W (DATA_W),
        .ID_W   (4),
        .DEPTH  (DDR_DEPTH)
    ) u_ddr (
        .clk     (clk),
        .rstn    (rst_n),
        // 写通道 tie
        .AWID    (4'b0),
        .AWADDR  (32'b0),
        .AWLEN   (8'b0),
        .AWBURST (2'b01),
        .AWVALID (1'b0),
        .AWREADY (),
        .WDATA   (128'b0),
        .WSTRB   (16'b0),
        .WLAST   (1'b0),
        .WVALID  (1'b0),
        .WREADY  (),
        .BID     (),
        .BRESP   (),
        .BVALID  (),
        .BREADY  (1'b1),
        // 读通道
        .ARID    (ar_id),
        .ARADDR  (ar_addr),
        .ARLEN   (ar_len),
        .ARBURST (ar_burst),
        .ARVALID (ar_valid),
        .ARREADY (ar_ready),
        .RID     (),
        .RDATA   (r_data),
        .RRESP   (r_resp),
        .RLAST   (r_last),
        .RVALID  (r_valid),
        .RREADY  (r_ready)
    );

    // =========================================================================
    // IFB SRAM (写端口由 idma_ctrl 驱动, 读端口我们 TB 自己读)
    // =========================================================================
    logic                  ifb_re;
    logic [SRAM_ADDR_W-1:0] ifb_raddr;
    logic [DATA_W-1:0]     ifb_rdata;

    sram_model #(
        .DEPTH      (IFB_DEPTH),
        .DATA_WIDTH (DATA_W)
    ) u_ifb (
        .clk    (clk),
        .we     (ifb_we),
        .waddr  (ifb_waddr),
        .wdata  (ifb_wdata),
        .re     (ifb_re),
        .raddr  (ifb_raddr),
        .rdata  (ifb_rdata)
    );

    // =========================================================================
    // 测试激励
    // =========================================================================
    initial begin
        rows_consumed = 16'd0;
        idma_start    = 1'b0;
        rst_n         = 1'b0;

        // DDR 预填: mem[i] = i (低 32-bit)
        for (int i = 0; i < TOTAL_BEATS; i++) begin
            u_ddr.mem[i] = {96'h0, 32'(i)};
        end

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("[tb] start idma_ctrl");
        idma_start = 1'b1;
        @(posedge clk);
        idma_start = 1'b0;

        // 等 done
        wait (idma_done == 1'b1);
        $display("[tb] idma_done seen at time %0t  rows_available=%0d  err=%b",
                 $time, rows_available, idma_err);

        if (idma_err) begin
            $display("[tb] FAIL  idma reported err");
            $fatal;
        end

        // 校验 IFB 内容
        begin
            int errs = 0;
            ifb_re = 1'b1;
            for (int i = 0; i < TOTAL_BEATS; i++) begin
                ifb_raddr = SRAM_ADDR_W'(i);
                @(posedge clk);
                #1;
                if (ifb_rdata !== {96'h0, 32'(i)}) begin
                    if (errs < 4) begin
                        $display("[tb] MISMATCH @%0d: got %032h expected %032h",
                                 i, ifb_rdata, {96'h0, 32'(i)});
                    end
                    errs++;
                end
            end
            ifb_re = 1'b0;

            if (errs == 0) begin
                $display("[tb] PASS  IFB[0..%0d] all match DDR", TOTAL_BEATS-1);
            end else begin
                $display("[tb] FAIL  %0d mismatches in %0d beats", errs, TOTAL_BEATS);
                $fatal;
            end
        end

        $finish;
    end

    initial begin
        #200us;
        $display("[tb] TIMEOUT");
        $fatal;
    end

    // ---- Debug instrumentation ----
    int n_cmd_fire = 0;
    int n_data_fire = 0;
    int n_data_last_fire = 0;
    int n_sts_fire = 0;
    int n_ar_fire = 0;
    int n_r_fire = 0;
    int n_r_last_fire = 0;

    always_ff @(posedge clk) if (rst_n) begin
        if (cmd_tvalid && cmd_tready) begin
            n_cmd_fire++;
            $display("[dbg %0t] CMD fire #%0d  saddr=%h  btt=%0d  type=%b",
                     $time, n_cmd_fire,
                     cmd_tdata[63:32], cmd_tdata[22:0], cmd_tdata[23]);
        end
        if (data_tvalid && data_tready) begin
            n_data_fire++;
            $display("[dbg %0t] DATA fire #%0d  tlast=%b tkeep=%h tdata[31:0]=%h",
                     $time, n_data_fire, data_tlast, data_tkeep, data_tdata[31:0]);
            if (data_tlast) begin
                n_data_last_fire++;
                $display("[dbg %0t] DATA last fire #%0d (total beats=%0d)",
                         $time, n_data_last_fire, n_data_fire);
            end
        end
        if (sts_tvalid && sts_tready) begin
            n_sts_fire++;
            $display("[dbg %0t] STS fire #%0d  data=%h", $time, n_sts_fire, sts_tdata);
        end
        if (ar_valid && ar_ready) begin
            n_ar_fire++;
            $display("[dbg %0t] AR fire #%0d  addr=%h len=%0d size=%0d burst=%b",
                     $time, n_ar_fire, ar_addr, ar_len, ar_size, ar_burst);
        end
        if (r_valid && r_ready) begin
            n_r_fire++;
            if (r_last) n_r_last_fire++;
        end
    end

    // 每 500ns 报一下计数器, 卡住时能看出堵在哪
    initial begin
        forever begin
            #500ns;
            $display("[dbg %0t] state=%0d cmd_fire=%0d ar_fire=%0d r_fire=%0d data_fire=%0d sts_fire=%0d busy=%b done=%b",
                     $time, u_idma_ctrl.state, n_cmd_fire, n_ar_fire, n_r_fire,
                     n_data_fire, n_sts_fire, idma_busy, idma_done);
        end
    end

endmodule
