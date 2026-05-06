`timescale 1ns/1ps

// =============================================================================
// conv_core_stub.sv  --  简化 ConvCore 模型 (用于 mesh PoC sim)
//
// 行为:
//   1. 接 mesh LOCAL port (1 AXIS slave + 1 AXIS master)
//   2. 收到 packet (axis_packet_rx) → 解 header → 写内部 4 区 SRAM (IFB/WB/SB/RDMA)
//   3. TB 拉 start_compute → 触发"compute"(简化: 直接 echo IFB → OFB)
//      实际行为: 把 IFB 区数据原封不动 + (data ^ core_id_xor) 后, 用 axis_packet_tx 发出去
//   4. 发完 → compute_done pulse
//
// 真 ConvCore 替代说明:
//   stub 的 4 区 SRAM 对应真 ConvCore 的 IFB/WB/Shortcut/RDMA SRAM (写方向一致)
//   stub 的 echo 对应真 ConvCore 的 line_buffer/mac_array/ofb_writer 计算 + 发包
//   compute 替换为真计算后, packet 收发部分不变
// =============================================================================

module conv_core_stub #(
    parameter int DATA_W  = 128,
    parameter int DEST_W  = 8,
    parameter int ADDR_W  = 20
)(
    input  logic                clk,
    input  logic                rst_n,
    input  logic [7:0]          core_id,        // TB 实例化时传 8-bit (避免 parameter cast)
    input  logic [15:0]         compute_delay,  // 收齐 IFB 后等 N 拍才发 OFM (模拟 compute 时间)

    // mesh LOCAL port (AXIS)
    input  logic                s_axis_tvalid,
    output logic                s_axis_tready,
    input  logic [DATA_W-1:0]   s_axis_tdata,
    input  logic                s_axis_tlast,
    input  logic [DEST_W-1:0]   s_axis_tdest,

    output logic                m_axis_tvalid,
    input  logic                m_axis_tready,
    output logic [DATA_W-1:0]   m_axis_tdata,
    output logic                m_axis_tlast,
    output logic [DEST_W-1:0]   m_axis_tdest,

    // 控制 (TB 驱动, 真 ConvCore 由 sequencer 触发)
    input  logic                start_compute,
    input  logic [DEST_W-1:0]   ofm_dst,            // OFM 发到哪个节点
    input  logic [3:0]          ofm_opcode,         // OFM packet 的 opcode (e.g. WRITE_DDR_OFB)
    output logic                compute_done,

    // 调试观察 (TB 检查用)
    output logic [3:0]          last_rx_opcode,
    output logic [15:0]         last_rx_burst_len,
    output logic [ADDR_W-1:0]   last_rx_addr
);

    // ----- axis_packet_rx → 写 SRAM -----
    logic                rx_we;
    logic [3:0]          rx_target;
    logic [ADDR_W-1:0]   rx_waddr;
    logic [DATA_W-1:0]   rx_wdata;
    logic                rx_packet_done;

    axis_packet_rx #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_rx (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),   .s_axis_tdata(s_axis_tdata),
        .s_axis_tdest(s_axis_tdest),
        .sram_we(rx_we), .sram_target(rx_target),
        .sram_waddr(rx_waddr), .sram_wdata(rx_wdata),
        .packet_done(rx_packet_done),
        .last_opcode(last_rx_opcode),
        .last_burst_len(last_rx_burst_len),
        .last_addr_offset(last_rx_addr)
    );

    // ----- 4 区内部 SRAM (mock, [target][addr]) -----
    // target: 0=IFB, 1=WB, 2=SB, 3=RDMA
    logic [DATA_W-1:0]   sram [0:3][0:1023];

    // 写口 (来自 axis_packet_rx)
    always_ff @(posedge clk) begin
        if (rx_we) sram[rx_target][rx_waddr[9:0]] <= rx_wdata;
    end

    // 记录最近收到的 IFB packet info, 给 echo 时用作 source
    logic [ADDR_W-1:0]   r_last_ifb_addr;
    logic [15:0]         r_last_ifb_len;
    logic                r_have_ifb;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_last_ifb_addr <= '0;
            r_last_ifb_len  <= '0;
            r_have_ifb      <= 1'b0;
        end else if (rx_packet_done && last_rx_opcode == 4'h0) begin
            r_last_ifb_addr <= last_rx_addr;
            r_last_ifb_len  <= last_rx_burst_len;
            r_have_ifb      <= 1'b1;
        end
    end

    // ----- 自动 compute 状态机 -----
    // IDLE → 收到 IFB 启动 counter → 数到 compute_delay → 触发 tx → 等 tx done → IDLE
    typedef enum logic [1:0] {C_IDLE, C_WAIT, C_TX} compute_state_t;
    compute_state_t  cstate;
    logic [15:0]     compute_cnt;
    logic            tx_start_int;
    logic            tx_done_int;
    logic            tx_busy_int;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cstate       <= C_IDLE;
            compute_cnt  <= '0;
            tx_start_int <= 1'b0;
        end else begin
            tx_start_int <= 1'b0;     // 默认 0, 在 WAIT 结束触发一拍 pulse
            case (cstate)
                C_IDLE: begin
                    if (rx_packet_done && last_rx_opcode == 4'h0) begin
                        cstate      <= C_WAIT;
                        compute_cnt <= '0;
                    end
                    if (start_compute) begin
                        // 外部强制触发 (兼容旧 TB)
                        cstate       <= C_TX;
                        tx_start_int <= 1'b1;
                    end
                end
                C_WAIT: begin
                    if (compute_cnt >= compute_delay) begin
                        cstate       <= C_TX;
                        tx_start_int <= 1'b1;
                    end else begin
                        compute_cnt <= compute_cnt + 16'd1;
                    end
                end
                C_TX: begin
                    if (tx_done_int) cstate <= C_IDLE;
                end
                default: cstate <= C_IDLE;
            endcase
        end
    end

    assign compute_done = tx_done_int;

    // ----- axis_packet_tx → 从 IFB SRAM 读 + 加 header + 发 -----
    logic                tx_sram_re;
    logic [ADDR_W-1:0]   tx_sram_raddr;
    logic [DATA_W-1:0]   tx_sram_rdata;

    always_ff @(posedge clk) begin
        if (tx_sram_re)
            tx_sram_rdata <= sram[0][tx_sram_raddr[9:0]] ^ {120'd0, core_id};
    end

    axis_packet_tx #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tx_start_int),
        .cfg_opcode(ofm_opcode),
        .cfg_sram_addr(r_last_ifb_addr),
        .cfg_burst_len(r_last_ifb_len),
        .cfg_tdest(ofm_dst),
        .busy(tx_busy_int),
        .done(tx_done_int),
        .sram_re(tx_sram_re), .sram_raddr(tx_sram_raddr), .sram_rdata(tx_sram_rdata),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready),
        .m_axis_tdata(m_axis_tdata),   .m_axis_tlast(m_axis_tlast),
        .m_axis_tdest(m_axis_tdest)
    );

endmodule
