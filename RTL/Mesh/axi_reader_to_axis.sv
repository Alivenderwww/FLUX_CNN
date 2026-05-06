`timescale 1ns/1ps

// =============================================================================
// axi_reader_to_axis.sv  --  ConvCore AXI master read 通道 → mesh READ_REQ packets
//
// NUMA 模式下 ConvCore 用全局地址主动拉数据 (替代 push 模型). 本模块是 ConvCore.bus_ar
// 通道到 mesh AXIS master 的桥, 把每个 AR 握手转成一个或多个 READ_REQ packet
// (按 mem 边界拆分, 自动支持跨 mem burst).
//
// 全局地址路由:
//   araddr[25:24] = mem ID (4 mem 系统)
//   araddr[23:0]  = mem 内 byte offset (16MB / mem)
//
// 跨 mem 边界拆分:
//   原始 burst: araddr 起, beats = arlen+1
//   每次到 mem 边界 (16MB align) 切一段, 每段一个 packet (tdest = 该段所在 mem ID)
//   axis_to_axi_read_resp 端按总 beats 累计计数拉 rlast (无视 packet 边界)
//
// READ_REQ packet (header only, 1 flit):
//   tdata[127:124] opcode = 0x1
//   tdata[123:104] addr20 = current_segment_addr[23:4]   (该子 burst 在 mem 内 word offset)
//   tdata[103:88]  burst_len = this_packet_len           (该子 burst beat 数)
//   tdata[87:80]   return_id = cfg_return_tdest          (响应送回这里)
//   tdest = {dst_y=0, dst_x = mem_id}
//   tlast = 1 (header only packet)
// =============================================================================

module axi_reader_to_axis #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ADDR_W = 32,
    parameter int ID_W   = 8
)(
    input  logic                 clk,
    input  logic                 rst_n,

    input  logic [ID_W-1:0]      s_axi_arid,
    input  logic [ADDR_W-1:0]    s_axi_araddr,
    input  logic [7:0]           s_axi_arlen,
    input  logic [2:0]           s_axi_arsize,
    input  logic [1:0]           s_axi_arburst,
    input  logic                 s_axi_arvalid,
    output logic                 s_axi_arready,

    output logic                 m_axis_tvalid,
    input  logic                 m_axis_tready,
    output logic [DATA_W-1:0]    m_axis_tdata,
    output logic                 m_axis_tlast,
    output logic [DEST_W-1:0]    m_axis_tdest,

    input  logic [DEST_W-1:0]    cfg_return_tdest,    // 自己 mesh 节点 (mem 收到后回复方向)
    output logic [15:0]          total_beats_out      // 跟踪用: 当前 outstanding burst 总 beats (axis_to_axi_read_resp 用)
);

    typedef enum logic [1:0] { S_IDLE, S_SEND } state_t;
    state_t st;

    logic [ADDR_W-1:0]  r_cur_addr;       // 当前子 burst 起点 (按 mem 边界推进)
    logic [15:0]        r_remaining;      // 还剩多少 beats 没发
    logic [15:0]        r_total_beats;    // burst 总 beats (= arlen+1, latch 一次)

    assign s_axi_arready    = (st == S_IDLE);
    assign total_beats_out  = r_total_beats;

    // ---- 算当前子 burst 在 mem 内的剩余空间 (到下个 16MB 边界) ----
    //   words_to_boundary = (mem 边界 - 当前 byte addr) / 16
    //   每 mem 16MB = 1<<24 byte, 边界 = (cur_addr & ~24'hFFFFFF) + 24'h1000000
    logic [24:0] bytes_to_boundary;        // 25-bit 包含 16MB 上限
    logic [15:0] words_to_boundary;
    assign bytes_to_boundary = 25'h1000000 - {1'b0, r_cur_addr[23:0]};
    assign words_to_boundary = bytes_to_boundary[23:4];   // / 16, 截到 16-bit

    // ---- 当前子 burst 长度 = min(remaining, words_to_boundary) ----
    logic [15:0] this_packet_len;
    assign this_packet_len = (r_remaining < words_to_boundary) ? r_remaining : words_to_boundary;

    // ---- packet header 编码 ----
    logic [3:0]  header_opcode;
    logic [19:0] header_addr20;
    logic [15:0] header_burst_len;
    assign header_opcode    = 4'h1;                          // READ_REQ
    assign header_addr20    = r_cur_addr[23:4];              // 当前子 burst 在 mem 内 word offset
    assign header_burst_len = this_packet_len;

    assign m_axis_tvalid = (st == S_SEND);
    assign m_axis_tlast  = (st == S_SEND);                   // header only, 1 flit
    assign m_axis_tdata  = {header_opcode,
                            header_addr20,
                            header_burst_len,
                            cfg_return_tdest,
                            80'd0};
    assign m_axis_tdest  = {4'd0, 2'd0, r_cur_addr[25:24]};  // mem 行 dst_y=0, dst_x = mem ID

    // FSM
    state_t st_next;
    always_comb begin
        st_next = st;
        case (st)
            S_IDLE : if (s_axi_arvalid && s_axi_arready) st_next = S_SEND;
            S_SEND : if (m_axis_tvalid && m_axis_tready) begin
                if (this_packet_len >= r_remaining) st_next = S_IDLE;       // 全发完
                else                                  st_next = S_SEND;     // 还有跨 mem 子 burst
            end
            default: st_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) st <= S_IDLE;
        else        st <= st_next;
    end

    // 锁 AR + 推进子 burst
    always_ff @(posedge clk) begin
        if (st == S_IDLE && s_axi_arvalid && s_axi_arready) begin
            // AR fire: 锁 burst 信息, 设第一段
            r_cur_addr    <= s_axi_araddr;
            r_remaining   <= {8'd0, s_axi_arlen} + 16'd1;
            r_total_beats <= {8'd0, s_axi_arlen} + 16'd1;
        end else if (st == S_SEND && m_axis_tvalid && m_axis_tready) begin
            // 子 packet 发完: 推进
            r_cur_addr  <= r_cur_addr + {16'd0, this_packet_len, 4'd0};   // += this_packet_len × 16 byte
            r_remaining <= r_remaining - this_packet_len;
        end
    end

endmodule
