`timescale 1ns/1ps

// =============================================================================
// mesh_4x2.sv  --  4×2 mesh wrapper (8 router_node, AIE-ML 风格 column 拓扑)
//
// 拓扑:
//   y=1   R(0,1)=4 ─ R(1,1)=5 ─ R(2,1)=6 ─ R(3,1)=7    ← ConvCore 行
//           │         │         │         │
//   y=0   R(0,0)=0 ─ R(1,0)=1 ─ R(2,0)=2 ─ R(3,0)=3    ← Mem Core 行
//          x=0       x=1       x=2       x=3
//
// 8 个 LOCAL ports 暴露:
//   idx 0..3 = Mem Core 0..3 (at y=0)
//   idx 4..7 = ConvCore 0..3 (at y=1)
//
// 互连:
//   E-W (per row): R(x,y) ↔ R(x+1,y)  for x=0..2, y=0..1
//   N-S (per col): R(x,0) ↔ R(x,1)    for x=0..3
//
// 路由 = XY (XY routing): 先 X 再 Y. ConvCore[i] 跟 Mem[i] 通信 = 1 hop (south).
// =============================================================================

module mesh_4x2 #(
    parameter int DATA_W = 128,
    parameter int DEST_W = 8
)(
    input  logic clk,
    input  logic rst_n,

    // 8 个 LOCAL slave (host → router LOCAL input)
    input  logic [7:0]            s_axis_local_tvalid,
    output logic [7:0]            s_axis_local_tready,
    input  logic [DATA_W-1:0]     s_axis_local_tdata  [8],
    input  logic [7:0]            s_axis_local_tlast,
    input  logic [DEST_W-1:0]     s_axis_local_tdest  [8],

    // 8 个 LOCAL master (router LOCAL output → host)
    output logic [7:0]            m_axis_local_tvalid,
    input  logic [7:0]            m_axis_local_tready,
    output logic [DATA_W-1:0]     m_axis_local_tdata  [8],
    output logic [7:0]            m_axis_local_tlast,
    output logic [DEST_W-1:0]     m_axis_local_tdest  [8]
);

    localparam int LOCAL_P = 0;
    localparam int NORTH_P = 1;
    localparam int SOUTH_P = 2;
    localparam int EAST_P  = 3;
    localparam int WEST_P  = 4;
    localparam int N_NODES = 8;

    // 每个 router 的 5-port 信号 (s/m_axis 视角)
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

    // ---------- E-W 互连 (per row) ----------
    // R(x,y).EAST ↔ R(x+1,y).WEST  for x=0..2, y=0..1
    // idx = y*4 + x, R(x+1,y) idx = y*4 + x + 1
    generate
        for (gi = 0; gi < 8; gi++) begin : g_ew
            // 仅 x=0..2 (即 idx % 4 != 3) 才有 east neighbor
            if ((gi % 4) != 3) begin
                // R(gi).EAST out → R(gi+1).WEST in
                assign r_in_tvalid[gi+1][WEST_P]  = r_out_tvalid[gi][EAST_P];
                assign r_in_tlast [gi+1][WEST_P]  = r_out_tlast [gi][EAST_P];
                assign r_in_tdata [gi+1][WEST_P]  = r_out_tdata [gi][EAST_P];
                assign r_in_tdest [gi+1][WEST_P]  = r_out_tdest [gi][EAST_P];
                assign r_out_tready[gi][EAST_P]   = r_in_tready[gi+1][WEST_P];

                // R(gi+1).WEST out → R(gi).EAST in
                assign r_in_tvalid[gi][EAST_P]    = r_out_tvalid[gi+1][WEST_P];
                assign r_in_tlast [gi][EAST_P]    = r_out_tlast [gi+1][WEST_P];
                assign r_in_tdata [gi][EAST_P]    = r_out_tdata [gi+1][WEST_P];
                assign r_in_tdest [gi][EAST_P]    = r_out_tdest [gi+1][WEST_P];
                assign r_out_tready[gi+1][WEST_P] = r_in_tready[gi][EAST_P];
            end
        end
    endgenerate

    // ---------- N-S 互连 (per col) ----------
    // R(x,0).NORTH ↔ R(x,1).SOUTH for x=0..3
    // idx 0..3 (y=0) ↔ idx 4..7 (y=1)
    generate
        for (gi = 0; gi < 4; gi++) begin : g_ns
            // R(x,0).NORTH out → R(x,1).SOUTH in
            assign r_in_tvalid[gi+4][SOUTH_P]  = r_out_tvalid[gi][NORTH_P];
            assign r_in_tlast [gi+4][SOUTH_P]  = r_out_tlast [gi][NORTH_P];
            assign r_in_tdata [gi+4][SOUTH_P]  = r_out_tdata [gi][NORTH_P];
            assign r_in_tdest [gi+4][SOUTH_P]  = r_out_tdest [gi][NORTH_P];
            assign r_out_tready[gi][NORTH_P]   = r_in_tready[gi+4][SOUTH_P];

            // R(x,1).SOUTH out → R(x,0).NORTH in
            assign r_in_tvalid[gi][NORTH_P]    = r_out_tvalid[gi+4][SOUTH_P];
            assign r_in_tlast [gi][NORTH_P]    = r_out_tlast [gi+4][SOUTH_P];
            assign r_in_tdata [gi][NORTH_P]    = r_out_tdata [gi+4][SOUTH_P];
            assign r_in_tdest [gi][NORTH_P]    = r_out_tdest [gi+4][SOUTH_P];
            assign r_out_tready[gi+4][SOUTH_P] = r_in_tready[gi][NORTH_P];
        end
    endgenerate

    // ---------- 边界 tie 0 ----------
    // x=0 列 WEST 边界: idx 0, 4
    // x=3 列 EAST 边界: idx 3, 7
    // y=0 行 SOUTH 边界: idx 0..3
    // y=1 行 NORTH 边界: idx 4..7
    generate
        for (gi = 0; gi < N_NODES; gi++) begin : g_bound
            // SOUTH 边界 (y=0 行)
            if (gi < 4) begin : g_south_tie
                assign r_in_tvalid[gi][SOUTH_P] = 1'b0;
                assign r_in_tlast [gi][SOUTH_P] = 1'b0;
                assign r_in_tdata [gi][SOUTH_P] = '0;
                assign r_in_tdest [gi][SOUTH_P] = '0;
                assign r_out_tready[gi][SOUTH_P] = 1'b1;
            end
            // NORTH 边界 (y=1 行)
            if (gi >= 4) begin : g_north_tie
                assign r_in_tvalid[gi][NORTH_P] = 1'b0;
                assign r_in_tlast [gi][NORTH_P] = 1'b0;
                assign r_in_tdata [gi][NORTH_P] = '0;
                assign r_in_tdest [gi][NORTH_P] = '0;
                assign r_out_tready[gi][NORTH_P] = 1'b1;
            end
            // WEST 边界 (x=0 列)
            if ((gi % 4) == 0) begin : g_west_tie
                assign r_in_tvalid[gi][WEST_P] = 1'b0;
                assign r_in_tlast [gi][WEST_P] = 1'b0;
                assign r_in_tdata [gi][WEST_P] = '0;
                assign r_in_tdest [gi][WEST_P] = '0;
                assign r_out_tready[gi][WEST_P] = 1'b1;
            end
            // EAST 边界 (x=3 列)
            if ((gi % 4) == 3) begin : g_east_tie
                assign r_in_tvalid[gi][EAST_P] = 1'b0;
                assign r_in_tlast [gi][EAST_P] = 1'b0;
                assign r_in_tdata [gi][EAST_P] = '0;
                assign r_in_tdest [gi][EAST_P] = '0;
                assign r_out_tready[gi][EAST_P] = 1'b1;
            end
        end
    endgenerate

    // ---------- 8 router 实例 ----------
    generate
        for (gi = 0; gi < N_NODES; gi++) begin : g_router
            localparam int X_P = gi % 4;
            localparam int Y_P = gi / 4;
            router_node #(
                .X_POS(X_P), .Y_POS(Y_P), .DATA_W(DATA_W), .DEST_W(DEST_W)
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
