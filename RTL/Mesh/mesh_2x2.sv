`timescale 1ns/1ps

// =============================================================================
// mesh_2x2.sv  --  2×2 mesh wrapper (4 router_node 互连, AXI4-Stream)
//
// 拓扑:
//
//     y=1   R(0,1) ──── R(1,1)
//             │           │
//     y=0   R(0,0) ──── R(1,0)
//           x=0          x=1
//
// 每个 router 5 ports {L=0, N=1, S=2, E=3, W=4}.
// 互连:
//   R(0,0).EAST  ↔ R(1,0).WEST
//   R(0,1).EAST  ↔ R(1,1).WEST
//   R(0,0).NORTH ↔ R(0,1).SOUTH
//   R(1,0).NORTH ↔ R(1,1).SOUTH
//   边界 ports tie 0.
//
// 4 个 LOCAL port 暴露给外部 (idx 0=R00, 1=R10, 2=R01, 3=R11).
// =============================================================================

module mesh_2x2 #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8
)(
    input  logic clk,
    input  logic rst_n,

    // 4 个 LOCAL slave (host → router LOCAL input)
    input  logic [3:0]            s_axis_local_tvalid,
    output logic [3:0]            s_axis_local_tready,
    input  logic [DATA_W-1:0]     s_axis_local_tdata  [4],
    input  logic [3:0]            s_axis_local_tlast,
    input  logic [DEST_W-1:0]     s_axis_local_tdest  [4],

    // 4 个 LOCAL master (router LOCAL output → host)
    output logic [3:0]            m_axis_local_tvalid,
    input  logic [3:0]            m_axis_local_tready,
    output logic [DATA_W-1:0]     m_axis_local_tdata  [4],
    output logic [3:0]            m_axis_local_tlast,
    output logic [DEST_W-1:0]     m_axis_local_tdest  [4]
);

    localparam int LOCAL_P = 0;
    localparam int NORTH_P = 1;
    localparam int SOUTH_P = 2;
    localparam int EAST_P  = 3;
    localparam int WEST_P  = 4;

    // 4 个 router 的 5-port 信号 (s/m_axis 视角)
    logic [4:0]            r_in_tvalid  [4];
    logic [4:0]            r_in_tready  [4];
    logic [4:0]            r_in_tlast   [4];
    logic [DATA_W-1:0]     r_in_tdata   [4][5];
    logic [DEST_W-1:0]     r_in_tdest   [4][5];

    logic [4:0]            r_out_tvalid [4];
    logic [4:0]            r_out_tready [4];
    logic [4:0]            r_out_tlast  [4];
    logic [DATA_W-1:0]     r_out_tdata  [4][5];
    logic [DEST_W-1:0]     r_out_tdest  [4][5];

    // ---------- LOCAL ports 接外部 ----------
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi++) begin : g_local
            assign r_in_tvalid[gi][LOCAL_P]   = s_axis_local_tvalid[gi];
            assign r_in_tlast [gi][LOCAL_P]   = s_axis_local_tlast[gi];
            assign r_in_tdata [gi][LOCAL_P]   = s_axis_local_tdata[gi];
            assign r_in_tdest [gi][LOCAL_P]   = s_axis_local_tdest[gi];
            assign s_axis_local_tready[gi]    = r_in_tready[gi][LOCAL_P];

            assign m_axis_local_tvalid[gi]    = r_out_tvalid[gi][LOCAL_P];
            assign m_axis_local_tlast [gi]    = r_out_tlast [gi][LOCAL_P];
            assign m_axis_local_tdata [gi]    = r_out_tdata [gi][LOCAL_P];
            assign m_axis_local_tdest [gi]    = r_out_tdest [gi][LOCAL_P];
            assign r_out_tready[gi][LOCAL_P]  = m_axis_local_tready[gi];
        end
    endgenerate

    // ---------- 互连: R00.E ↔ R10.W ----------
    assign r_in_tvalid[1][WEST_P]  = r_out_tvalid[0][EAST_P];
    assign r_in_tlast [1][WEST_P]  = r_out_tlast [0][EAST_P];
    assign r_in_tdata [1][WEST_P]  = r_out_tdata [0][EAST_P];
    assign r_in_tdest [1][WEST_P]  = r_out_tdest [0][EAST_P];
    assign r_out_tready[0][EAST_P] = r_in_tready[1][WEST_P];

    assign r_in_tvalid[0][EAST_P]  = r_out_tvalid[1][WEST_P];
    assign r_in_tlast [0][EAST_P]  = r_out_tlast [1][WEST_P];
    assign r_in_tdata [0][EAST_P]  = r_out_tdata [1][WEST_P];
    assign r_in_tdest [0][EAST_P]  = r_out_tdest [1][WEST_P];
    assign r_out_tready[1][WEST_P] = r_in_tready[0][EAST_P];

    // ---------- 互连: R01.E ↔ R11.W ----------
    assign r_in_tvalid[3][WEST_P]  = r_out_tvalid[2][EAST_P];
    assign r_in_tlast [3][WEST_P]  = r_out_tlast [2][EAST_P];
    assign r_in_tdata [3][WEST_P]  = r_out_tdata [2][EAST_P];
    assign r_in_tdest [3][WEST_P]  = r_out_tdest [2][EAST_P];
    assign r_out_tready[2][EAST_P] = r_in_tready[3][WEST_P];

    assign r_in_tvalid[2][EAST_P]  = r_out_tvalid[3][WEST_P];
    assign r_in_tlast [2][EAST_P]  = r_out_tlast [3][WEST_P];
    assign r_in_tdata [2][EAST_P]  = r_out_tdata [3][WEST_P];
    assign r_in_tdest [2][EAST_P]  = r_out_tdest [3][WEST_P];
    assign r_out_tready[3][WEST_P] = r_in_tready[2][EAST_P];

    // ---------- 互连: R00.N ↔ R01.S ----------
    assign r_in_tvalid[2][SOUTH_P] = r_out_tvalid[0][NORTH_P];
    assign r_in_tlast [2][SOUTH_P] = r_out_tlast [0][NORTH_P];
    assign r_in_tdata [2][SOUTH_P] = r_out_tdata [0][NORTH_P];
    assign r_in_tdest [2][SOUTH_P] = r_out_tdest [0][NORTH_P];
    assign r_out_tready[0][NORTH_P] = r_in_tready[2][SOUTH_P];

    assign r_in_tvalid[0][NORTH_P] = r_out_tvalid[2][SOUTH_P];
    assign r_in_tlast [0][NORTH_P] = r_out_tlast [2][SOUTH_P];
    assign r_in_tdata [0][NORTH_P] = r_out_tdata [2][SOUTH_P];
    assign r_in_tdest [0][NORTH_P] = r_out_tdest [2][SOUTH_P];
    assign r_out_tready[2][SOUTH_P] = r_in_tready[0][NORTH_P];

    // ---------- 互连: R10.N ↔ R11.S ----------
    assign r_in_tvalid[3][SOUTH_P] = r_out_tvalid[1][NORTH_P];
    assign r_in_tlast [3][SOUTH_P] = r_out_tlast [1][NORTH_P];
    assign r_in_tdata [3][SOUTH_P] = r_out_tdata [1][NORTH_P];
    assign r_in_tdest [3][SOUTH_P] = r_out_tdest [1][NORTH_P];
    assign r_out_tready[1][NORTH_P] = r_in_tready[3][SOUTH_P];

    assign r_in_tvalid[1][NORTH_P] = r_out_tvalid[3][SOUTH_P];
    assign r_in_tlast [1][NORTH_P] = r_out_tlast [3][SOUTH_P];
    assign r_in_tdata [1][NORTH_P] = r_out_tdata [3][SOUTH_P];
    assign r_in_tdest [1][NORTH_P] = r_out_tdest [3][SOUTH_P];
    assign r_out_tready[3][SOUTH_P] = r_in_tready[1][NORTH_P];

    // ---------- 边界 ports tie 0 ----------
    // R00 (0): WEST + SOUTH 边界
    assign r_in_tvalid[0][WEST_P]  = 1'b0;
    assign r_in_tlast [0][WEST_P]  = 1'b0;
    assign r_in_tdata [0][WEST_P]  = '0;
    assign r_in_tdest [0][WEST_P]  = '0;
    assign r_out_tready[0][WEST_P] = 1'b1;
    assign r_in_tvalid[0][SOUTH_P] = 1'b0;
    assign r_in_tlast [0][SOUTH_P] = 1'b0;
    assign r_in_tdata [0][SOUTH_P] = '0;
    assign r_in_tdest [0][SOUTH_P] = '0;
    assign r_out_tready[0][SOUTH_P] = 1'b1;
    // R10 (1): EAST + SOUTH 边界
    assign r_in_tvalid[1][EAST_P]  = 1'b0;
    assign r_in_tlast [1][EAST_P]  = 1'b0;
    assign r_in_tdata [1][EAST_P]  = '0;
    assign r_in_tdest [1][EAST_P]  = '0;
    assign r_out_tready[1][EAST_P] = 1'b1;
    assign r_in_tvalid[1][SOUTH_P] = 1'b0;
    assign r_in_tlast [1][SOUTH_P] = 1'b0;
    assign r_in_tdata [1][SOUTH_P] = '0;
    assign r_in_tdest [1][SOUTH_P] = '0;
    assign r_out_tready[1][SOUTH_P] = 1'b1;
    // R01 (2): WEST + NORTH 边界
    assign r_in_tvalid[2][WEST_P]  = 1'b0;
    assign r_in_tlast [2][WEST_P]  = 1'b0;
    assign r_in_tdata [2][WEST_P]  = '0;
    assign r_in_tdest [2][WEST_P]  = '0;
    assign r_out_tready[2][WEST_P] = 1'b1;
    assign r_in_tvalid[2][NORTH_P] = 1'b0;
    assign r_in_tlast [2][NORTH_P] = 1'b0;
    assign r_in_tdata [2][NORTH_P] = '0;
    assign r_in_tdest [2][NORTH_P] = '0;
    assign r_out_tready[2][NORTH_P] = 1'b1;
    // R11 (3): EAST + NORTH 边界
    assign r_in_tvalid[3][EAST_P]  = 1'b0;
    assign r_in_tlast [3][EAST_P]  = 1'b0;
    assign r_in_tdata [3][EAST_P]  = '0;
    assign r_in_tdest [3][EAST_P]  = '0;
    assign r_out_tready[3][EAST_P] = 1'b1;
    assign r_in_tvalid[3][NORTH_P] = 1'b0;
    assign r_in_tlast [3][NORTH_P] = 1'b0;
    assign r_in_tdata [3][NORTH_P] = '0;
    assign r_in_tdest [3][NORTH_P] = '0;
    assign r_out_tready[3][NORTH_P] = 1'b1;

    // ---------- 4 router 实例 ----------
    router_node #(.X_POS(0), .Y_POS(0), .DATA_W(DATA_W), .DEST_W(DEST_W)) u_r00 (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(r_in_tvalid[0]),  .s_axis_tready(r_in_tready[0]),
        .s_axis_tlast(r_in_tlast[0]),    .s_axis_tdata(r_in_tdata[0]),
        .s_axis_tdest(r_in_tdest[0]),
        .m_axis_tvalid(r_out_tvalid[0]), .m_axis_tready(r_out_tready[0]),
        .m_axis_tlast(r_out_tlast[0]),   .m_axis_tdata(r_out_tdata[0]),
        .m_axis_tdest(r_out_tdest[0])
    );
    router_node #(.X_POS(1), .Y_POS(0), .DATA_W(DATA_W), .DEST_W(DEST_W)) u_r10 (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(r_in_tvalid[1]),  .s_axis_tready(r_in_tready[1]),
        .s_axis_tlast(r_in_tlast[1]),    .s_axis_tdata(r_in_tdata[1]),
        .s_axis_tdest(r_in_tdest[1]),
        .m_axis_tvalid(r_out_tvalid[1]), .m_axis_tready(r_out_tready[1]),
        .m_axis_tlast(r_out_tlast[1]),   .m_axis_tdata(r_out_tdata[1]),
        .m_axis_tdest(r_out_tdest[1])
    );
    router_node #(.X_POS(0), .Y_POS(1), .DATA_W(DATA_W), .DEST_W(DEST_W)) u_r01 (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(r_in_tvalid[2]),  .s_axis_tready(r_in_tready[2]),
        .s_axis_tlast(r_in_tlast[2]),    .s_axis_tdata(r_in_tdata[2]),
        .s_axis_tdest(r_in_tdest[2]),
        .m_axis_tvalid(r_out_tvalid[2]), .m_axis_tready(r_out_tready[2]),
        .m_axis_tlast(r_out_tlast[2]),   .m_axis_tdata(r_out_tdata[2]),
        .m_axis_tdest(r_out_tdest[2])
    );
    router_node #(.X_POS(1), .Y_POS(1), .DATA_W(DATA_W), .DEST_W(DEST_W)) u_r11 (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(r_in_tvalid[3]),  .s_axis_tready(r_in_tready[3]),
        .s_axis_tlast(r_in_tlast[3]),    .s_axis_tdata(r_in_tdata[3]),
        .s_axis_tdest(r_in_tdest[3]),
        .m_axis_tvalid(r_out_tvalid[3]), .m_axis_tready(r_out_tready[3]),
        .m_axis_tlast(r_out_tlast[3]),   .m_axis_tdata(r_out_tdata[3]),
        .m_axis_tdest(r_out_tdest[3])
    );

endmodule
