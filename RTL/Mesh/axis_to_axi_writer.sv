`timescale 1ns/1ps

// =============================================================================
// axis_to_axi_writer.sv  --  AXI4-Stream packet → AXI4 master burst write
//
// 用途: mesh 节点的 LOCAL AXIS 端 → 桥接到 ConvCore 的 rmt_ifb_axi_slave (AXI4 slave).
// ConvCore 不动, 通过现有 cross-core IFB push 通道接收 mesh packet 的数据.
//
// 行为:
//   1. 收 axis packet header flit (tlast=0 第一个): 解 opcode + addr + burst_len
//   2. 发 axi aw (awaddr = header.addr × 16 字节, awlen = burst_len-1, awsize=4)
//   3. 流式 forward axis body → axi w (1:1, tdata → wdata, tlast → wlast)
//   4. 等 axi b 完成
//
// 假设: BUS_DATA_W = 128 (= 16 byte/beat). 一个 axis packet = 一个 axi burst.
// =============================================================================

module axis_to_axi_writer #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ADDR_W = 32,
    parameter int ID_W   = 6
)(
    input  logic                clk,
    input  logic                rst_n,

    // AXIS slave (从 mesh router LOCAL master 来)
    input  logic                s_axis_tvalid,
    output logic                s_axis_tready,
    input  logic [DATA_W-1:0]   s_axis_tdata,
    input  logic                s_axis_tlast,
    input  logic [DEST_W-1:0]   s_axis_tdest,    // 不用 (已经路由到 LOCAL)

    // 配置: axi 写到哪 (上层提供 base_addr, 跟 packet header.addr 拼起来)
    input  logic [ADDR_W-1:0]   cfg_base_addr,   // 跟 packet addr 加起来 = axi awaddr

    // AXI4 master write 通道 (接 core_top.rmt_ifb_aw* / w* / b*)
    output logic [ID_W-1:0]     m_axi_awid,
    output logic [ADDR_W-1:0]   m_axi_awaddr,
    output logic [7:0]          m_axi_awlen,
    output logic [2:0]          m_axi_awsize,
    output logic [1:0]          m_axi_awburst,
    output logic                m_axi_awlock,
    output logic [3:0]          m_axi_awcache,
    output logic [2:0]          m_axi_awprot,
    output logic [3:0]          m_axi_awqos,
    output logic                m_axi_awvalid,
    input  logic                m_axi_awready,

    output logic [DATA_W-1:0]   m_axi_wdata,
    output logic [DATA_W/8-1:0] m_axi_wstrb,
    output logic                m_axi_wlast,
    output logic                m_axi_wvalid,
    input  logic                m_axi_wready,

    input  logic [ID_W-1:0]     m_axi_bid,
    input  logic [1:0]          m_axi_bresp,
    input  logic                m_axi_bvalid,
    output logic                m_axi_bready
);

    typedef enum logic [1:0] {S_IDLE, S_AW, S_W, S_B} state_t;
    state_t st;

    logic [ADDR_W-1:0]   r_addr;          // base + packet header addr (byte addr)
    logic [7:0]          r_awlen;         // burst_len - 1

    // 头部解析: packet header flit
    logic [3:0]          h_opcode;
    logic [19:0]         h_addr20;
    logic [15:0]         h_burst_len;
    assign h_opcode    = s_axis_tdata[127:124];
    assign h_addr20    = s_axis_tdata[123:104];
    assign h_burst_len = s_axis_tdata[103:88];

    // ---------------- AXI 默认信号 ----------------
    assign m_axi_awid    = '0;
    assign m_axi_awsize  = 3'd4;       // 16 byte (BUS_DATA_W=128)
    assign m_axi_awburst = 2'b01;      // INCR
    assign m_axi_awlock  = 1'b0;
    assign m_axi_awcache = 4'b0011;    // normal non-cacheable bufferable
    assign m_axi_awprot  = 3'b000;
    assign m_axi_awqos   = 4'b0;
    assign m_axi_wstrb   = '1;
    assign m_axi_bready  = (st == S_B);
    assign m_axi_awaddr  = r_addr;
    assign m_axi_awlen   = r_awlen;
    assign m_axi_awvalid = (st == S_AW);

    // axis ↔ axi w forward
    // S_W 状态: axis tvalid → axi wvalid; axi wready → axis tready
    assign m_axi_wdata   = s_axis_tdata;
    assign m_axi_wlast   = s_axis_tlast;
    assign m_axi_wvalid  = (st == S_W) && s_axis_tvalid;
    assign s_axis_tready = (st == S_IDLE) ||                              // header 收
                           ((st == S_W) && m_axi_wready);                 // body fwd

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st     <= S_IDLE;
            r_addr <= '0;
            r_awlen<= '0;
        end else begin
            case (st)
                S_IDLE: if (s_axis_tvalid && s_axis_tready) begin
                    // 收 header flit
                    r_addr  <= cfg_base_addr + ({{(ADDR_W-20){1'b0}}, h_addr20} << 4); // word→byte
                    r_awlen <= 8'(h_burst_len - 16'd1);
                    st      <= S_AW;
                end
                S_AW: if (m_axi_awvalid && m_axi_awready) begin
                    st <= S_W;
                end
                S_W: if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
                    st <= S_B;
                end
                S_B: if (m_axi_bvalid && m_axi_bready) begin
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
