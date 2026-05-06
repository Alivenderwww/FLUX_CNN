`timescale 1ns/1ps

// =============================================================================
// router_node.sv  --  2D Mesh router (5-port, wormhole, XY routing, AXI4-Stream)
//
// 拓扑: 2D mesh, 每节点 5 ports {Local=0, North=1, South=2, East=3, West=4}.
// 协议: AXI4-Stream (s_axis/m_axis_t{valid,ready,data,last,dest}).
//
// tdest 编码 (8 bit):
//   tdest[7:4] = dst_y[3:0]
//   tdest[3:0] = dst_x[3:0]
//
// 路由 (XY): 比较 dst_y/dst_x 跟节点 (X_POS, Y_POS), 决定 output port:
//   if dst_x > X_POS → EAST
//   if dst_x < X_POS → WEST
//   if dst_x == X_POS:
//       if dst_y > Y_POS → NORTH
//       if dst_y < Y_POS → SOUTH
//       else            → LOCAL
//
// Wormhole: first flit (tlast 没来过) 决定路径, 后续 flit 跟随同 output 直到 tlast.
//   每 flit 都带 tdest (AXIS 协议特性), 但 router 只在 packet 第一拍 latch output port.
//
// 缓冲: 每 input 1-deep skid buffer.
// 仲裁: 每 output round-robin.
// =============================================================================

module router_node #(
    parameter int X_POS  = 0,
    parameter int Y_POS  = 0,
    parameter int DATA_W = 128,
    parameter int DEST_W = 8           // tdest 字段宽度 (4-bit y + 4-bit x)
)(
    input  logic clk,
    input  logic rst_n,

    // 5 input AXIS slave ports {L=0, N=1, S=2, E=3, W=4}
    input  logic [4:0]            s_axis_tvalid,
    output logic [4:0]            s_axis_tready,
    input  logic [DATA_W-1:0]     s_axis_tdata  [5],
    input  logic [4:0]            s_axis_tlast,
    input  logic [DEST_W-1:0]     s_axis_tdest  [5],

    // 5 output AXIS master ports {L=0, N=1, S=2, E=3, W=4}
    output logic [4:0]            m_axis_tvalid,
    input  logic [4:0]            m_axis_tready,
    output logic [DATA_W-1:0]     m_axis_tdata  [5],
    output logic [4:0]            m_axis_tlast,
    output logic [DEST_W-1:0]     m_axis_tdest  [5]
);

    localparam int LOCAL = 0;
    localparam int NORTH = 1;
    localparam int SOUTH = 2;
    localparam int EAST  = 3;
    localparam int WEST  = 4;

    // ---------------- Per-input skid buffer ----------------
    logic [4:0]            buf_valid;
    logic [4:0]            buf_tlast;
    logic [DATA_W-1:0]     buf_tdata [5];
    logic [DEST_W-1:0]     buf_tdest [5];
    logic [4:0]            buf_pop;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int i = 0; i < 5; i++) begin
                buf_valid[i] <= 1'b0;
                buf_tlast[i] <= 1'b0;
                buf_tdata[i] <= '0;
                buf_tdest[i] <= '0;
            end
        end else begin
            for (int i = 0; i < 5; i++) begin
                if (buf_pop[i]) buf_valid[i] <= 1'b0;
                if (s_axis_tvalid[i] && s_axis_tready[i]) begin
                    buf_valid[i] <= 1'b1;
                    buf_tlast[i] <= s_axis_tlast[i];
                    buf_tdata[i] <= s_axis_tdata[i];
                    buf_tdest[i] <= s_axis_tdest[i];
                end
            end
        end
    end

    // s_axis_tready: skid buf 空 OR 当前 cycle 会 pop
    always_comb begin
        for (int i = 0; i < 5; i++)
            s_axis_tready[i] = !buf_valid[i] || buf_pop[i];
    end

    // ---------------- XY routing ----------------
    function automatic logic [2:0] xy_route(input logic [3:0] dst_x,
                                             input logic [3:0] dst_y);
        logic signed [4:0] sx_dst, sx_my;
        logic signed [4:0] sy_dst, sy_my;
        sx_dst = {1'b0, dst_x}; sx_my = X_POS;
        sy_dst = {1'b0, dst_y}; sy_my = Y_POS;
        if (sx_dst > sx_my) return 3'd3;     // EAST
        if (sx_dst < sx_my) return 3'd4;     // WEST
        if (sy_dst > sy_my) return 3'd1;     // NORTH
        if (sy_dst < sy_my) return 3'd2;     // SOUTH
        return 3'd0;                          // LOCAL
    endfunction

    // 给每个 input flit 算目标 output port (基于 tdest)
    logic [2:0] flit_target [5];
    always_comb begin
        for (int i = 0; i < 5; i++)
            flit_target[i] = xy_route(buf_tdest[i][3:0], buf_tdest[i][7:4]);
    end

    // ---------------- Per-input wormhole state ----------------
    // route_active[i] = 1: packet 在传输中(已经 latch first flit, 等 tlast)
    // route_lock[i]  : 锁定的 output port
    logic [4:0] route_active;
    logic [2:0] route_lock [5];

    // effective target: active 时用 lock, 否则用 flit_target (新 packet 的 first flit)
    logic [2:0] in_target [5];
    always_comb begin
        for (int i = 0; i < 5; i++)
            in_target[i] = route_active[i] ? route_lock[i] : flit_target[i];
    end

    // ---------------- Round-robin arbitration per output (with wormhole lock) ----------------
    logic [2:0] rr_priority;
    always_ff @(posedge clk) begin
        if (!rst_n) rr_priority <= 3'd0;
        else if (|m_axis_tvalid)
                    rr_priority <= (rr_priority == 3'd4) ? 3'd0 : rr_priority + 3'd1;
    end

    // 每 output port 一个 wormhole lock: 一旦选了 input, 锁到该 input 的 tlast 为止.
    // 防止多 input 的 packet 在同 output 上交错破坏 wormhole.
    logic [4:0] out_active;        // 1: output op 正在转发 packet
    logic [2:0] out_locked_in [5]; // 锁定的 input idx

    logic [4:0] out_grant_valid;
    logic [2:0] out_grant_in [5];

    always_comb begin
        for (int op = 0; op < 5; op++) begin
            out_grant_valid[op] = 1'b0;
            out_grant_in[op]    = 3'd0;
        end
        for (int op = 0; op < 5; op++) begin
            if (out_active[op]) begin
                // wormhole locked: 强制选 locked input (即使 buf_valid=0 也保留, 等 input 来)
                if (buf_valid[out_locked_in[op]] &&
                    in_target[out_locked_in[op]] == op[2:0]) begin
                    out_grant_valid[op] = 1'b1;
                    out_grant_in[op]    = out_locked_in[op];
                end
                // 如果 locked input 暂时没数据, output 啥也不发 (等)
            end else begin
                // output idle, round-robin 选 input
                for (int k = 0; k < 5; k++) begin
                    automatic int i = (rr_priority + k) % 5;
                    if (!out_grant_valid[op] &&
                        buf_valid[i] && (in_target[i] == op[2:0])) begin
                        out_grant_valid[op] = 1'b1;
                        out_grant_in[op]    = i[2:0];
                    end
                end
            end
        end
    end

    // out_active update: fire 时若不是 tlast 则 active, tlast 时清.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            out_active <= 5'd0;
            for (int op = 0; op < 5; op++) out_locked_in[op] <= 3'd0;
        end else begin
            for (int op = 0; op < 5; op++) begin
                if (out_grant_valid[op] && m_axis_tready[op]) begin
                    if (!out_active[op]) begin
                        // 第一拍 fire (新 packet), lock input
                        out_locked_in[op] <= out_grant_in[op];
                        out_active[op]    <= !buf_tlast[out_grant_in[op]];
                    end else if (buf_tlast[out_locked_in[op]]) begin
                        // 最后一拍 (tlast) fire, 释放 lock
                        out_active[op] <= 1'b0;
                    end
                end
            end
        end
    end

    // ---------------- Output port: forward 选中 input 的 flit ----------------
    always_comb begin
        for (int op = 0; op < 5; op++) begin
            if (out_grant_valid[op]) begin
                m_axis_tvalid[op] = 1'b1;
                m_axis_tdata[op]  = buf_tdata[out_grant_in[op]];
                m_axis_tlast[op]  = buf_tlast[out_grant_in[op]];
                m_axis_tdest[op]  = buf_tdest[out_grant_in[op]];
            end else begin
                m_axis_tvalid[op] = 1'b0;
                m_axis_tdata[op]  = '0;
                m_axis_tlast[op]  = 1'b0;
                m_axis_tdest[op]  = '0;
            end
        end
    end

    // ---------------- Pop signal ----------------
    always_comb begin
        for (int i = 0; i < 5; i++) begin
            buf_pop[i] = 1'b0;
            for (int op = 0; op < 5; op++) begin
                if (out_grant_valid[op] && out_grant_in[op] == i[2:0] && m_axis_tready[op])
                    buf_pop[i] = 1'b1;
            end
        end
    end

    // ---------------- Wormhole lock state ----------------
    // first flit (route_active==0) 通过时 latch lock + 设 active
    // tlast 通过时 clear active (可 pipeline 下一个 packet)
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            route_active <= 5'd0;
            for (int i = 0; i < 5; i++) route_lock[i] <= 3'd0;
        end else begin
            for (int i = 0; i < 5; i++) begin
                if (buf_valid[i] && buf_pop[i]) begin
                    if (!route_active[i]) begin
                        route_lock[i]   <= flit_target[i];
                        route_active[i] <= !buf_tlast[i];
                    end else if (buf_tlast[i]) begin
                        route_active[i] <= 1'b0;
                    end
                end
            end
        end
    end

    // ---------------- Profiling 计数器 (per port) ----------------
    // in_fire[p]:  该 input port 收到一个 flit 的周期数
    // out_fire[p]: 该 output port 发出一个 flit 的周期数
    int in_fire  [5];
    int out_fire [5];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            for (int p = 0; p < 5; p++) begin
                in_fire[p]  <= 0;
                out_fire[p] <= 0;
            end
        end else begin
            for (int p = 0; p < 5; p++) begin
                if (s_axis_tvalid[p] && s_axis_tready[p]) in_fire[p]  <= in_fire[p]  + 1;
                if (m_axis_tvalid[p] && m_axis_tready[p]) out_fire[p] <= out_fire[p] + 1;
            end
        end
    end

endmodule
