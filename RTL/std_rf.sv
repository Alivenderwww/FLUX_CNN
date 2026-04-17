`timescale 1ns/1ps

// Standard Register File (Std_RF)
//
// 存储阵列 ram 是数据路径，无复位：上游 we 未拉高前任何读出值都被下游 gate，
// 上电 X 不影响可观测行为（§6.1）。仿真为避免 X 污染，用 initial 清 0。
module std_rf #(
    parameter int DATA_WIDTH = 8,
    parameter int DEPTH      = 32
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Write port
    input  logic                          we,
    input  logic [$clog2(DEPTH)-1:0]      waddr,
    input  logic [DATA_WIDTH-1:0]         wdata,

    // Read port
    input  logic [$clog2(DEPTH)-1:0]      raddr,
    output logic [DATA_WIDTH-1:0]         rdata
);

    // ram 数据路径，无复位，无初值：仿真时上电 X，但使用契约保证先写后读
    // （wgt_buffer 的 S_LOAD 阶段灌满后 S_COMPUTE 才读），不产生观测污染。
    logic [DATA_WIDTH-1:0] ram [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (we) ram[waddr] <= wdata;
        else    ram[waddr] <= ram[waddr];
    end

    always_comb begin
        rdata = ram[raddr];
    end

    // synthesis translate_off
    int total_write_ops;
    logic first_write_logged;

    always_ff @(posedge clk) begin
        if (!rst_n) total_write_ops <= 0;
        else if (we) total_write_ops <= total_write_ops + 1;
        else         total_write_ops <= total_write_ops;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            first_write_logged <= 1'b0;
        end else if (we && !first_write_logged) begin
            $display("Time=%0t, %m written! waddr=%0d, wdata=%x", $time, waddr, wdata);
            first_write_logged <= 1'b1;
        end else begin
            first_write_logged <= first_write_logged;
        end
    end
    // synthesis translate_on

endmodule
