`timescale 1ns/1ps

// =============================================================================
// axi_writer_to_axis.sv  --  AXI4 slave (write only) → AXI4-Stream packet
//
// 用途: ConvCore 的 ODMA (axi master) 输出 → 桥转 axis packet → mesh
//   ConvCore 不动, ODMA 仍然主动发 axi burst write, 桥负责包成 packet 出 mesh.
//
// 行为:
//   1. 接 axi master 的 aw + w + b
//   2. aw fire 时锁 awaddr + awlen, 同时发 axis header flit (opcode/addr/burst_len)
//   3. 流式 forward axi w → axis body (1:1, wlast → tlast)
//   4. wlast fire 后回 axi b
//
// Packet header 编码 (跟 axis_packet_rx 解析一致):
//   tdata[127:124] = opcode (NUMA: 0x0 = WRITE; legacy push 模式可配 cfg_opcode)
//   tdata[123:104] = awaddr[23:4]  (mem 内 word offset, 20-bit)
//   tdata[103:88]  = awlen + 1     (burst_len 总数)
//   tdata[87:80]   = return_id (WRITE 不用, 0)
//
// tdest 路由 (NUMA 模式):
//   tdest = {4'd0, awaddr[25:24], 2'd0}  // 自动按 awaddr[25:24] 解码 mem ID
//   (awaddr[25:24] 是全局地址中的 mem ID 段, 4 mem 系统占 2 bit)
//
// Legacy push 模式: cfg_use_addr_route=0 时用 cfg_tdest 单值 (Phase 6 兼容)
// =============================================================================

module axi_writer_to_axis #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ADDR_W = 32,
    parameter int ID_W   = 6
)(
    input  logic                clk,
    input  logic                rst_n,

    // AXI4 slave (write only): 接 ConvCore.bus_aw* / bus_w* / bus_b* (ODMA 部分)
    input  logic [ID_W-1:0]     s_axi_awid,
    input  logic [ADDR_W-1:0]   s_axi_awaddr,
    input  logic [7:0]          s_axi_awlen,
    input  logic [2:0]          s_axi_awsize,
    input  logic [1:0]          s_axi_awburst,
    input  logic                s_axi_awvalid,
    output logic                s_axi_awready,

    input  logic [DATA_W-1:0]   s_axi_wdata,
    input  logic [DATA_W/8-1:0] s_axi_wstrb,
    input  logic                s_axi_wlast,
    input  logic                s_axi_wvalid,
    output logic                s_axi_wready,

    output logic [ID_W-1:0]     s_axi_bid,
    output logic [1:0]          s_axi_bresp,
    output logic                s_axi_bvalid,
    input  logic                s_axi_bready,

    // 配置:
    //   cfg_opcode: packet header opcode (默认 0x0 NUMA WRITE; 旧 push 模式 0x5)
    //   cfg_use_addr_route: 1 = NUMA 模式 (tdest 由 awaddr[25:24] 解码),
    //                       0 = legacy push (tdest = cfg_tdest 单值)
    //   cfg_tdest: legacy push 模式下的固定目标
    input  logic [3:0]          cfg_opcode,
    input  logic                cfg_use_addr_route,
    input  logic [DEST_W-1:0]   cfg_tdest,

    // AXIS master (接 mesh router LOCAL slave)
    output logic                m_axis_tvalid,
    input  logic                m_axis_tready,
    output logic [DATA_W-1:0]   m_axis_tdata,
    output logic                m_axis_tlast,
    output logic [DEST_W-1:0]   m_axis_tdest
);

    typedef enum logic [1:0] {S_IDLE, S_HEADER, S_BODY, S_B} state_t;
    state_t st;

    logic [ID_W-1:0]     r_awid;
    logic [ADDR_W-1:0]   r_awaddr;
    logic [7:0]          r_awlen;          // burst_len - 1
    logic [DATA_W-1:0]   header_flit;

    // 头部 flit 内容
    assign header_flit = {cfg_opcode,
                           r_awaddr[23:4],                    // [123:104] 20-bit word offset
                           {8'd0, r_awlen} + 16'd1,           // [103:88]  burst_len = awlen+1
                           88'd0};

    // ---------------- AXI side ----------------
    assign s_axi_awready = (st == S_IDLE);
    // body 拍: forward s_axi_w → m_axis (1:1)
    assign s_axi_wready  = (st == S_BODY) && m_axis_tready;
    assign s_axi_bid     = r_awid;
    assign s_axi_bresp   = 2'b00;
    assign s_axi_bvalid  = (st == S_B);

    // ---------------- AXIS side ----------------
    // tdest 路由: NUMA 用 awaddr[25:24] 解码 mem ID, legacy push 用 cfg_tdest
    logic [DEST_W-1:0] computed_tdest;
    assign computed_tdest = cfg_use_addr_route
                             ? {4'd0, 2'd0, r_awaddr[25:24]}      // mem 行 dst_y=0, dst_x = awaddr[25:24]
                             : cfg_tdest;

    always_comb begin
        m_axis_tvalid = 1'b0;
        m_axis_tdata  = '0;
        m_axis_tlast  = 1'b0;
        m_axis_tdest  = computed_tdest;
        case (st)
            S_HEADER: begin
                m_axis_tvalid = 1'b1;
                m_axis_tdata  = header_flit;
                m_axis_tlast  = 1'b0;     // 多 flit packet, header 不是 last
            end
            S_BODY: begin
                m_axis_tvalid = s_axi_wvalid;
                m_axis_tdata  = s_axi_wdata;
                m_axis_tlast  = s_axi_wlast;
            end
            default: ;
        endcase
    end

    // ---------------- FSM ----------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            r_awid   <= '0;
            r_awaddr <= '0;
            r_awlen  <= '0;
        end else begin
            case (st)
                S_IDLE: if (s_axi_awvalid && s_axi_awready) begin
                    r_awid   <= s_axi_awid;
                    r_awaddr <= s_axi_awaddr;
                    r_awlen  <= s_axi_awlen;
                    st       <= S_HEADER;
                end
                S_HEADER: if (m_axis_tvalid && m_axis_tready) begin
                    st <= S_BODY;
                end
                S_BODY: if (s_axi_wvalid && s_axi_wready && s_axi_wlast) begin
                    st <= S_B;
                end
                S_B: if (s_axi_bvalid && s_axi_bready) begin
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
