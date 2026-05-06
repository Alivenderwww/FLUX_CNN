`timescale 1ns/1ps

// =============================================================================
// axis_packet_tx.sv  --  从 SRAM 读出 → AXI4-Stream master 输出
//
// 发送一个 packet (header flit + N body flit + tail flit):
//   触发: start=1 + 配置 (opcode/addr/burst_len/dst_x/dst_y)
//   header: 自己生成 (含 opcode + addr_offset + burst_len)
//   body:  从 SRAM 读 burst_len 个 word, 流式输出
//   tail:  最后一个 body 带 tlast=1
//
// SRAM 读口由上层 mux 控制 (本模块只发 sram_re + sram_raddr).
// 假设 SRAM 1 cycle 延迟: cycle N 发 raddr → cycle N+1 拿 rdata.
// =============================================================================

module axis_packet_tx #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8,
    parameter int ADDR_W = 20
)(
    input  logic                clk,
    input  logic                rst_n,

    // 控制接口 (上层触发, 一次一个 packet)
    input  logic                start,           // pulse 开始一个 packet
    input  logic [3:0]          cfg_opcode,
    input  logic [ADDR_W-1:0]   cfg_sram_addr,   // SRAM 起始读地址 (word)
    input  logic [15:0]         cfg_burst_len,   // body flit 数
    input  logic [DEST_W-1:0]   cfg_tdest,       // 目标节点 (dst_y, dst_x)
    output logic                busy,
    output logic                done,            // pulse: tail 发出后

    // SRAM 读口
    output logic                sram_re,
    output logic [ADDR_W-1:0]   sram_raddr,
    input  logic [DATA_W-1:0]   sram_rdata,

    // AXIS master 输出 (接 mesh router LOCAL slave)
    output logic                m_axis_tvalid,
    input  logic                m_axis_tready,
    output logic [DATA_W-1:0]   m_axis_tdata,
    output logic                m_axis_tlast,
    output logic [DEST_W-1:0]   m_axis_tdest
);

    typedef enum logic [1:0] {S_IDLE, S_HEADER, S_BODY} state_t;
    state_t st;

    logic [3:0]          r_opcode;
    logic [ADDR_W-1:0]   r_sram_base;
    logic [15:0]         r_burst_len;
    logic [DEST_W-1:0]   r_tdest;
    logic [15:0]         r_body_cnt;       // 已发 body flit 数
    logic [ADDR_W-1:0]   r_raddr;          // 当前 SRAM 读地址
    logic                r_rdata_valid;    // sram_rdata 这拍有效 (上拍发了 re)

    // header flit 数据组装
    // packet header.addr 字段是 20 bit, 取 r_sram_base 低 20 bit (够覆盖 16 MB / 16 byte = 1 M word)
    logic [DATA_W-1:0] header_flit;
    logic [19:0]       header_addr20;
    assign header_addr20 = (ADDR_W >= 20) ? r_sram_base[19:0]
                                          : {{(20){1'b0}} | r_sram_base};
    assign header_flit = {r_opcode,
                           header_addr20,                        // [123:104]
                           r_burst_len,                          // [103:88]
                           88'd0};                               // [87:0]

    assign busy = (st != S_IDLE);

    // SRAM 读发起: 在 HEADER 拍 (准备好 body 第 0 拍的数据), 跟在 BODY 中(未发完)
    // 简化: HEADER 拍发 raddr=base, 下一拍 (BODY 第 0 拍) sram_rdata 出现
    //       BODY 第 N 拍 fire 后, 发 raddr=base+N+1 准备第 N+1 拍数据
    always_comb begin
        sram_re = 1'b0;
        sram_raddr = r_raddr;
        case (st)
            S_HEADER: begin
                // 准备 body 第 0 拍数据
                if (m_axis_tready) begin
                    sram_re = 1'b1;
                    sram_raddr = r_sram_base;
                end
            end
            S_BODY: begin
                // 当前 body 拍 fire 后, 准备下一拍
                if (m_axis_tvalid && m_axis_tready && r_body_cnt < r_burst_len - 16'd1) begin
                    sram_re = 1'b1;
                    sram_raddr = r_sram_base + ADDR_W'(r_body_cnt + 16'd1);
                end
            end
            default: ;
        endcase
    end

    // 输出 AXIS
    always_comb begin
        m_axis_tvalid = 1'b0;
        m_axis_tdata  = '0;
        m_axis_tlast  = 1'b0;
        m_axis_tdest  = r_tdest;
        case (st)
            S_HEADER: begin
                m_axis_tvalid = 1'b1;
                m_axis_tdata  = header_flit;
                m_axis_tlast  = (r_burst_len == 16'd0);   // 0-body packet: header 即 tail
            end
            S_BODY: begin
                m_axis_tvalid = r_rdata_valid;
                m_axis_tdata  = sram_rdata;
                m_axis_tlast  = (r_body_cnt == r_burst_len - 16'd1);
            end
            default: ;
        endcase
    end

    // FSM
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            st <= S_IDLE;
            r_opcode    <= '0;
            r_sram_base <= '0;
            r_burst_len <= '0;
            r_tdest     <= '0;
            r_body_cnt  <= '0;
            r_raddr     <= '0;
            r_rdata_valid <= 1'b0;
            done        <= 1'b0;
        end else begin
            done <= 1'b0;
            case (st)
                S_IDLE: if (start) begin
                    r_opcode    <= cfg_opcode;
                    r_sram_base <= cfg_sram_addr;
                    r_burst_len <= cfg_burst_len;
                    r_tdest     <= cfg_tdest;
                    r_body_cnt  <= '0;
                    r_rdata_valid <= 1'b0;
                    st <= S_HEADER;
                end
                S_HEADER: if (m_axis_tvalid && m_axis_tready) begin
                    if (r_burst_len == 16'd0) begin
                        // 0-body packet (header 即 tail)
                        st <= S_IDLE;
                        done <= 1'b1;
                    end else begin
                        // header fire 同拍发了 sram_re=1 + raddr=base, 下拍 BODY 用
                        r_rdata_valid <= 1'b1;
                        st <= S_BODY;
                    end
                end
                S_BODY: begin
                    if (m_axis_tvalid && m_axis_tready) begin
                        r_body_cnt <= r_body_cnt + 16'd1;
                        if (r_body_cnt == r_burst_len - 16'd1) begin
                            // tail fire
                            st <= S_IDLE;
                            done <= 1'b1;
                            r_rdata_valid <= 1'b0;
                        end else begin
                            // 准备下一拍 body data: 上面 always_comb 已发 sram_re
                            r_rdata_valid <= 1'b1;
                        end
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
