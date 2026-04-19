`timescale 1ns/1ps

// =============================================================================
// desc_fifo.sv  --  Descriptor FIFO (DFF-based, depth 32, width 256)
//
// 服务于 DFE (writer) → Sequencer (reader) 的 descriptor 交付。
// 简单 synchronous FIFO，同一时钟域；wr / rd 端口独立 enable。
//
// 同拍 push+pop（full 时 push 被拒，empty 时 pop 被拒）不支持 —— DFE 看 full
// 反压，Sequencer 看 empty 等待。
//
// 复位：控制路径（wr_ptr / rd_ptr / count）同步复位（§6.1 影响下游有效信号）。
// 存储阵列不复位（§6 数据路径）。
// =============================================================================

module desc_fifo #(
    parameter int DEPTH = 32,
    parameter int WIDTH = 256
)(
    input  logic                clk,
    input  logic                rst_n,

    // Write port (from DFE)
    input  logic                wr_en,
    input  logic [WIDTH-1:0]    wr_data,
    output logic                full,

    // Read port (to Sequencer)
    input  logic                rd_en,
    output logic [WIDTH-1:0]    rd_data,
    output logic                empty,

    // Observability
    output logic [$clog2(DEPTH+1)-1:0] count
);

    localparam int PTR_W = $clog2(DEPTH);
    localparam int CNT_W = $clog2(DEPTH+1);

    logic [WIDTH-1:0] mem [0:DEPTH-1];
    logic [PTR_W-1:0] wr_ptr, rd_ptr;
    logic [CNT_W-1:0] r_count;

    assign full  = (r_count == CNT_W'(DEPTH));
    assign empty = (r_count == '0);
    assign count = r_count;

    // Read data (combinational — FIFO output reflects head entry immediately)
    assign rd_data = mem[rd_ptr];

    logic do_wr, do_rd;
    assign do_wr = wr_en && !full;
    assign do_rd = rd_en && !empty;

    // Storage (no reset, §6)
    always_ff @(posedge clk) begin
        if (do_wr) mem[wr_ptr] <= wr_data;
    end

    // Pointers + count (control path, reset §6.1)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr  <= '0;
            rd_ptr  <= '0;
            r_count <= '0;
        end else begin
            if (do_wr) wr_ptr <= (wr_ptr == PTR_W'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
            if (do_rd) rd_ptr <= (rd_ptr == PTR_W'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;

            case ({do_wr, do_rd})
                2'b10: r_count <= r_count + 1'b1;
                2'b01: r_count <= r_count - 1'b1;
                default: r_count <= r_count;
            endcase
        end
    end

endmodule
