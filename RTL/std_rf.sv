`timescale 1ns/1ps

// Standard Register File (Std_RF)
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

    logic [DATA_WIDTH-1:0] ram [0:DEPTH-1];

    // Write logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < DEPTH; i++) begin
                ram[i] <= '0;
            end
        end else if (we) begin
            ram[waddr] <= wdata;
        end
    end

    // Read logic
    always_comb begin
        rdata = ram[raddr];
    end

    // synthesis translate_off
    int total_write_ops = 0;
    int write_cnt = 0;
    always_ff @(posedge clk) begin
        if (rst_n && we) begin
            total_write_ops++;
            if (write_cnt < 1) begin
                $display("Time=%0t, %m written! waddr=%0d, wdata=%x", $time, waddr, wdata);
                write_cnt++;
                if (write_cnt == 3) $display("%m: ... (Further write messages suppressed)");
            end
        end
    end
    // synthesis translate_on

endmodule
