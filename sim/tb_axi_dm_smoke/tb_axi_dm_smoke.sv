`timescale 1ns/1ps
// =============================================================================
// tb_axi_dm_smoke.sv  --  AXI DataMover IP smoke test
//
// 目的: 仅验证 ModelSim 能正确加载 Vivado 仿真库 + 完成 axi_dm 的跨语言 (SV→VHDL)
// 例化和 elaboration. 不发任何真正的 mm2s/s2mm 命令, 复位后 ready 拉起, 跑几百拍.
//
// 通过条件:
//   1. vcom/vlog 全部 0 error
//   2. vsim elaborate 全部 0 error (找到 axi_datamover_v5_1_30 / fifo_generator_*
//      等依赖库的实体)
//   3. 复位 deassert 后 IP 不报 $fatal/$error, mm2s_err / s2mm_err 保持 0
//
// 不验证:
//   - 实际 cmd 流送入 / 数据搬运行为 (留给后续 functional TB)
// =============================================================================

module tb_axi_dm_smoke;

    logic clk = 1'b0;
    logic rst_n = 1'b0;

    initial forever #5 clk = ~clk;   // 100MHz

    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
    end

    // ---- 全部 input port 默认值 ----
    // cmd 流 / data 流统一拉低 (没人发命令)
    // ready 类信号拉高 (随时接受 IP 输出)
    logic        s_axis_mm2s_cmd_tvalid    = 1'b0;
    logic [71:0] s_axis_mm2s_cmd_tdata     = 72'b0;
    logic        m_axis_mm2s_sts_tready    = 1'b1;
    logic        m_axi_mm2s_arready        = 1'b1;
    logic [127:0] m_axi_mm2s_rdata         = 128'b0;
    logic [1:0]  m_axi_mm2s_rresp          = 2'b00;
    logic        m_axi_mm2s_rlast          = 1'b0;
    logic        m_axi_mm2s_rvalid         = 1'b0;
    logic        m_axis_mm2s_tready        = 1'b1;

    logic        s_axis_s2mm_cmd_tvalid    = 1'b0;
    logic [71:0] s_axis_s2mm_cmd_tdata     = 72'b0;
    logic        m_axis_s2mm_sts_tready    = 1'b1;
    logic        m_axi_s2mm_awready        = 1'b1;
    logic        m_axi_s2mm_wready         = 1'b1;
    logic [1:0]  m_axi_s2mm_bresp          = 2'b00;
    logic        m_axi_s2mm_bvalid         = 1'b0;
    logic [127:0] s_axis_s2mm_tdata        = 128'b0;
    logic [15:0] s_axis_s2mm_tkeep         = 16'b0;
    logic        s_axis_s2mm_tlast         = 1'b0;
    logic        s_axis_s2mm_tvalid        = 1'b0;

    // ---- IP outputs (我们不消费, 留 wire 即可) ----
    wire         mm2s_err;
    wire         s_axis_mm2s_cmd_tready;
    wire         m_axis_mm2s_sts_tvalid;
    wire [7:0]   m_axis_mm2s_sts_tdata;
    wire [0:0]   m_axis_mm2s_sts_tkeep;
    wire         m_axis_mm2s_sts_tlast;
    wire [3:0]   m_axi_mm2s_arid;
    wire [31:0]  m_axi_mm2s_araddr;
    wire [7:0]   m_axi_mm2s_arlen;
    wire [2:0]   m_axi_mm2s_arsize;
    wire [1:0]   m_axi_mm2s_arburst;
    wire [2:0]   m_axi_mm2s_arprot;
    wire [3:0]   m_axi_mm2s_arcache;
    wire [3:0]   m_axi_mm2s_aruser;
    wire         m_axi_mm2s_arvalid;
    wire         m_axi_mm2s_rready;
    wire [127:0] m_axis_mm2s_tdata;
    wire [15:0]  m_axis_mm2s_tkeep;
    wire         m_axis_mm2s_tlast;
    wire         m_axis_mm2s_tvalid;

    wire         s2mm_err;
    wire         s_axis_s2mm_cmd_tready;
    wire         m_axis_s2mm_sts_tvalid;
    wire [7:0]   m_axis_s2mm_sts_tdata;
    wire [0:0]   m_axis_s2mm_sts_tkeep;
    wire         m_axis_s2mm_sts_tlast;
    wire [3:0]   m_axi_s2mm_awid;
    wire [31:0]  m_axi_s2mm_awaddr;
    wire [7:0]   m_axi_s2mm_awlen;
    wire [2:0]   m_axi_s2mm_awsize;
    wire [1:0]   m_axi_s2mm_awburst;
    wire [2:0]   m_axi_s2mm_awprot;
    wire [3:0]   m_axi_s2mm_awcache;
    wire [3:0]   m_axi_s2mm_awuser;
    wire         m_axi_s2mm_awvalid;
    wire [127:0] m_axi_s2mm_wdata;
    wire [15:0]  m_axi_s2mm_wstrb;
    wire         m_axi_s2mm_wlast;
    wire         m_axi_s2mm_wvalid;
    wire         m_axi_s2mm_bready;
    wire         s_axis_s2mm_tready;

    // ---- DUT (基于 Vivado 生成的 axi_dm.veo 模板) ----
    axi_dm u_axi_dm (
        .m_axi_mm2s_aclk            (clk),
        .m_axi_mm2s_aresetn         (rst_n),
        .mm2s_err                   (mm2s_err),
        .m_axis_mm2s_cmdsts_aclk    (clk),
        .m_axis_mm2s_cmdsts_aresetn (rst_n),
        .s_axis_mm2s_cmd_tvalid     (s_axis_mm2s_cmd_tvalid),
        .s_axis_mm2s_cmd_tready     (s_axis_mm2s_cmd_tready),
        .s_axis_mm2s_cmd_tdata      (s_axis_mm2s_cmd_tdata),
        .m_axis_mm2s_sts_tvalid     (m_axis_mm2s_sts_tvalid),
        .m_axis_mm2s_sts_tready     (m_axis_mm2s_sts_tready),
        .m_axis_mm2s_sts_tdata      (m_axis_mm2s_sts_tdata),
        .m_axis_mm2s_sts_tkeep      (m_axis_mm2s_sts_tkeep),
        .m_axis_mm2s_sts_tlast      (m_axis_mm2s_sts_tlast),
        .m_axi_mm2s_arid            (m_axi_mm2s_arid),
        .m_axi_mm2s_araddr          (m_axi_mm2s_araddr),
        .m_axi_mm2s_arlen           (m_axi_mm2s_arlen),
        .m_axi_mm2s_arsize          (m_axi_mm2s_arsize),
        .m_axi_mm2s_arburst         (m_axi_mm2s_arburst),
        .m_axi_mm2s_arprot          (m_axi_mm2s_arprot),
        .m_axi_mm2s_arcache         (m_axi_mm2s_arcache),
        .m_axi_mm2s_aruser          (m_axi_mm2s_aruser),
        .m_axi_mm2s_arvalid         (m_axi_mm2s_arvalid),
        .m_axi_mm2s_arready         (m_axi_mm2s_arready),
        .m_axi_mm2s_rdata           (m_axi_mm2s_rdata),
        .m_axi_mm2s_rresp           (m_axi_mm2s_rresp),
        .m_axi_mm2s_rlast           (m_axi_mm2s_rlast),
        .m_axi_mm2s_rvalid          (m_axi_mm2s_rvalid),
        .m_axi_mm2s_rready          (m_axi_mm2s_rready),
        .m_axis_mm2s_tdata          (m_axis_mm2s_tdata),
        .m_axis_mm2s_tkeep          (m_axis_mm2s_tkeep),
        .m_axis_mm2s_tlast          (m_axis_mm2s_tlast),
        .m_axis_mm2s_tvalid         (m_axis_mm2s_tvalid),
        .m_axis_mm2s_tready         (m_axis_mm2s_tready),
        .m_axi_s2mm_aclk            (clk),
        .m_axi_s2mm_aresetn         (rst_n),
        .s2mm_err                   (s2mm_err),
        .m_axis_s2mm_cmdsts_awclk   (clk),
        .m_axis_s2mm_cmdsts_aresetn (rst_n),
        .s_axis_s2mm_cmd_tvalid     (s_axis_s2mm_cmd_tvalid),
        .s_axis_s2mm_cmd_tready     (s_axis_s2mm_cmd_tready),
        .s_axis_s2mm_cmd_tdata      (s_axis_s2mm_cmd_tdata),
        .m_axis_s2mm_sts_tvalid     (m_axis_s2mm_sts_tvalid),
        .m_axis_s2mm_sts_tready     (m_axis_s2mm_sts_tready),
        .m_axis_s2mm_sts_tdata      (m_axis_s2mm_sts_tdata),
        .m_axis_s2mm_sts_tkeep      (m_axis_s2mm_sts_tkeep),
        .m_axis_s2mm_sts_tlast      (m_axis_s2mm_sts_tlast),
        .m_axi_s2mm_awid            (m_axi_s2mm_awid),
        .m_axi_s2mm_awaddr          (m_axi_s2mm_awaddr),
        .m_axi_s2mm_awlen           (m_axi_s2mm_awlen),
        .m_axi_s2mm_awsize          (m_axi_s2mm_awsize),
        .m_axi_s2mm_awburst         (m_axi_s2mm_awburst),
        .m_axi_s2mm_awprot          (m_axi_s2mm_awprot),
        .m_axi_s2mm_awcache         (m_axi_s2mm_awcache),
        .m_axi_s2mm_awuser          (m_axi_s2mm_awuser),
        .m_axi_s2mm_awvalid         (m_axi_s2mm_awvalid),
        .m_axi_s2mm_awready         (m_axi_s2mm_awready),
        .m_axi_s2mm_wdata           (m_axi_s2mm_wdata),
        .m_axi_s2mm_wstrb           (m_axi_s2mm_wstrb),
        .m_axi_s2mm_wlast           (m_axi_s2mm_wlast),
        .m_axi_s2mm_wvalid          (m_axi_s2mm_wvalid),
        .m_axi_s2mm_wready          (m_axi_s2mm_wready),
        .m_axi_s2mm_bresp           (m_axi_s2mm_bresp),
        .m_axi_s2mm_bvalid          (m_axi_s2mm_bvalid),
        .m_axi_s2mm_bready          (m_axi_s2mm_bready),
        .s_axis_s2mm_tdata          (s_axis_s2mm_tdata),
        .s_axis_s2mm_tkeep          (s_axis_s2mm_tkeep),
        .s_axis_s2mm_tlast          (s_axis_s2mm_tlast),
        .s_axis_s2mm_tvalid         (s_axis_s2mm_tvalid),
        .s_axis_s2mm_tready         (s_axis_s2mm_tready)
    );

    initial begin
        $display("[smoke] start");
        @(posedge rst_n);   // 等复位释放
        repeat (200) @(posedge clk);
        // 复位释放后 IP 应该 idle, err 信号保持 0
        if (mm2s_err !== 1'b0 || s2mm_err !== 1'b0) begin
            $display("[smoke] FAIL  mm2s_err=%b s2mm_err=%b", mm2s_err, s2mm_err);
            $fatal;
        end
        $display("[smoke] PASS  cmd_tready: mm2s=%b s2mm=%b   err: mm2s=%b s2mm=%b",
                 s_axis_mm2s_cmd_tready, s_axis_s2mm_cmd_tready,
                 mm2s_err, s2mm_err);
        $finish;
    end

    initial begin
        #100us;
        $display("[smoke] TIMEOUT");
        $fatal;
    end

endmodule
