`timescale 1ns/1ps

// =============================================================================
// mesh_5x1.sv  --  5×1 mesh (一行 5 节点, 1D mesh)
//
// 拓扑: x=0..4, y=0
//       R(0,0) ─ R(1,0) ─ R(2,0) ─ R(3,0) ─ R(4,0)
//
// 每节点 5-port router_node, 但 1D mesh 下 N/S 边界 tie 0, 只用 E/W/L.
// 5 个 LOCAL ports 暴露 (idx 0..4 = x=0..4).
// =============================================================================

module mesh_5x1 #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8
)(
    input  logic clk,
    input  logic rst_n,

    // 5 个 LOCAL slave (host → router LOCAL input)
    input  logic [4:0]            s_axis_local_tvalid,
    output logic [4:0]            s_axis_local_tready,
    input  logic [DATA_W-1:0]     s_axis_local_tdata  [5],
    input  logic [4:0]            s_axis_local_tlast,
    input  logic [DEST_W-1:0]     s_axis_local_tdest  [5],

    // 5 个 LOCAL master (router LOCAL output → host)
    output logic [4:0]            m_axis_local_tvalid,
    input  logic [4:0]            m_axis_local_tready,
    output logic [DATA_W-1:0]     m_axis_local_tdata  [5],
    output logic [4:0]            m_axis_local_tlast,
    output logic [DEST_W-1:0]     m_axis_local_tdest  [5]
);

    localparam int LOCAL_P = 0;
    localparam int NORTH_P = 1;
    localparam int SOUTH_P = 2;
    localparam int EAST_P  = 3;
    localparam int WEST_P  = 4;
    localparam int N_NODES = 5;

    logic [4:0]            r_in_tvalid  [N_NODES];
    logic [4:0]            r_in_tready  [N_NODES];
    logic [4:0]            r_in_tlast   [N_NODES];
    logic [DATA_W-1:0]     r_in_tdata   [N_NODES][5];
    logic [DEST_W-1:0]     r_in_tdest   [N_NODES][5];

    logic [4:0]            r_out_tvalid [N_NODES];
    logic [4:0]            r_out_tready [N_NODES];
    logic [4:0]            r_out_tlast  [N_NODES];
    logic [DATA_W-1:0]     r_out_tdata  [N_NODES][5];
    logic [DEST_W-1:0]     r_out_tdest  [N_NODES][5];

    // ---------- LOCAL ports 接外部 ----------
    genvar gi;
    generate
        for (gi = 0; gi < N_NODES; gi++) begin : g_local
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

    // ---------- E-W 互连 (相邻) ----------
    // R(i).EAST ↔ R(i+1).WEST  for i = 0..N-2
    generate
        for (gi = 0; gi < N_NODES - 1; gi++) begin : g_ew
            // R(i).EAST out → R(i+1).WEST in
            assign r_in_tvalid[gi+1][WEST_P]  = r_out_tvalid[gi][EAST_P];
            assign r_in_tlast [gi+1][WEST_P]  = r_out_tlast [gi][EAST_P];
            assign r_in_tdata [gi+1][WEST_P]  = r_out_tdata [gi][EAST_P];
            assign r_in_tdest [gi+1][WEST_P]  = r_out_tdest [gi][EAST_P];
            assign r_out_tready[gi][EAST_P]   = r_in_tready[gi+1][WEST_P];

            // R(i+1).WEST out → R(i).EAST in
            assign r_in_tvalid[gi][EAST_P]    = r_out_tvalid[gi+1][WEST_P];
            assign r_in_tlast [gi][EAST_P]    = r_out_tlast [gi+1][WEST_P];
            assign r_in_tdata [gi][EAST_P]    = r_out_tdata [gi+1][WEST_P];
            assign r_in_tdest [gi][EAST_P]    = r_out_tdest [gi+1][WEST_P];
            assign r_out_tready[gi+1][WEST_P] = r_in_tready[gi][EAST_P];
        end
    endgenerate

    // ---------- 边界 tie 0 ----------
    // R(0).WEST 边界 + R(N-1).EAST 边界
    assign r_in_tvalid[0][WEST_P]              = 1'b0;
    assign r_in_tlast [0][WEST_P]              = 1'b0;
    assign r_in_tdata [0][WEST_P]              = '0;
    assign r_in_tdest [0][WEST_P]              = '0;
    assign r_out_tready[0][WEST_P]             = 1'b1;

    assign r_in_tvalid[N_NODES-1][EAST_P]      = 1'b0;
    assign r_in_tlast [N_NODES-1][EAST_P]      = 1'b0;
    assign r_in_tdata [N_NODES-1][EAST_P]      = '0;
    assign r_in_tdest [N_NODES-1][EAST_P]      = '0;
    assign r_out_tready[N_NODES-1][EAST_P]     = 1'b1;

    // 全部 N/S 边界 tie 0 (1D mesh)
    generate
        for (gi = 0; gi < N_NODES; gi++) begin : g_ns_tie
            assign r_in_tvalid[gi][NORTH_P]   = 1'b0;
            assign r_in_tlast [gi][NORTH_P]   = 1'b0;
            assign r_in_tdata [gi][NORTH_P]   = '0;
            assign r_in_tdest [gi][NORTH_P]   = '0;
            assign r_out_tready[gi][NORTH_P]  = 1'b1;

            assign r_in_tvalid[gi][SOUTH_P]   = 1'b0;
            assign r_in_tlast [gi][SOUTH_P]   = 1'b0;
            assign r_in_tdata [gi][SOUTH_P]   = '0;
            assign r_in_tdest [gi][SOUTH_P]   = '0;
            assign r_out_tready[gi][SOUTH_P]  = 1'b1;
        end
    endgenerate

    // ---------- 5 router 实例 ----------
    generate
        for (gi = 0; gi < N_NODES; gi++) begin : g_router
            router_node #(
                .X_POS(gi), .Y_POS(0), .DATA_W(DATA_W), .DEST_W(DEST_W)
            ) u_r (
                .clk(clk), .rst_n(rst_n),
                .s_axis_tvalid(r_in_tvalid[gi]),  .s_axis_tready(r_in_tready[gi]),
                .s_axis_tlast(r_in_tlast[gi]),    .s_axis_tdata(r_in_tdata[gi]),
                .s_axis_tdest(r_in_tdest[gi]),
                .m_axis_tvalid(r_out_tvalid[gi]), .m_axis_tready(r_out_tready[gi]),
                .m_axis_tlast(r_out_tlast[gi]),   .m_axis_tdata(r_out_tdata[gi]),
                .m_axis_tdest(r_out_tdest[gi])
            );
        end
    endgenerate

endmodule
