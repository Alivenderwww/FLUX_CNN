`timescale 1ns/1ps

// =============================================================================
// mm2s_arb.sv  --  AXI DataMover MM2S 通道仲裁器 (idma_dm + wdma_dm 二选一)
//
// 单条 cmd in-flight 串行仲裁: 同一时间只允许一个 user (idma/wdma) 占用 MM2S
// 通道, 完整跑完 cmd → data stream (含 tlast) → sts 后再释放. 简单、无饿死、
// 不需要 owner FIFO. 性能代价: idma 和 wdma 的 cmd 不能并行流, 等价于 wdma
// 一次性预加载权重期间 idma 暂停 (实际系统里 wdma 先发, idma 在 wdma 完后才开始
// streaming, 不会真的并发等待).
//
// 优先级: 当两路同时有 cmd_tvalid 时, idma 先 (流式负载更敏感; wdma 是一次性).
//
// 数据/状态分流: 用 1-bit 'owner' 寄存器记录当前 in-flight cmd 是 idma 还是
// wdma; 在 OWN 状态下把 mm2s_data 路由给 owner, mm2s_sts 同样. tlast_fire 后
// 等 sts_fire 来才释放 (保证下游收到状态).
//
// 状态机:
//   S_IDLE    : 没人在用. 看哪路有 cmd_tvalid; 锁 owner, 进 S_CMD
//   S_CMD     : 转发该 owner 的 cmd 到 axi_dm. cmd_fire → S_DATA
//   S_DATA    : 转发 axi_dm.data 到 owner. tlast_fire → S_STS
//   S_STS     : 等 axi_dm.sts 来转发到 owner. sts_fire → S_IDLE (释放)
// =============================================================================

module mm2s_arb #(
    parameter int DATA_W = 128
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- 端口 0: idma_dm ----
    input  logic                    idma_cmd_tvalid,
    output logic                    idma_cmd_tready,
    input  logic [71:0]             idma_cmd_tdata,
    output logic                    idma_data_tvalid,
    input  logic                    idma_data_tready,
    output logic [DATA_W-1:0]       idma_data_tdata,
    output logic [DATA_W/8-1:0]     idma_data_tkeep,
    output logic                    idma_data_tlast,
    output logic                    idma_sts_tvalid,
    input  logic                    idma_sts_tready,
    output logic [7:0]              idma_sts_tdata,

    // ---- 端口 1: wdma_dm ----
    input  logic                    wdma_cmd_tvalid,
    output logic                    wdma_cmd_tready,
    input  logic [71:0]             wdma_cmd_tdata,
    output logic                    wdma_data_tvalid,
    input  logic                    wdma_data_tready,
    output logic [DATA_W-1:0]       wdma_data_tdata,
    output logic [DATA_W/8-1:0]     wdma_data_tkeep,
    output logic                    wdma_data_tlast,
    output logic                    wdma_sts_tvalid,
    input  logic                    wdma_sts_tready,
    output logic [7:0]              wdma_sts_tdata,

    // ---- 上游: axi_dm.MM2S ----
    output logic                    mm2s_cmd_tvalid,
    input  logic                    mm2s_cmd_tready,
    output logic [71:0]             mm2s_cmd_tdata,
    input  logic                    mm2s_data_tvalid,
    output logic                    mm2s_data_tready,
    input  logic [DATA_W-1:0]       mm2s_data_tdata,
    input  logic [DATA_W/8-1:0]     mm2s_data_tkeep,
    input  logic                    mm2s_data_tlast,
    input  logic                    mm2s_sts_tvalid,
    output logic                    mm2s_sts_tready,
    input  logic [7:0]              mm2s_sts_tdata
);

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE = 2'd0,
        S_CMD  = 2'd1,
        S_DATA = 2'd2,
        S_STS  = 2'd3
    } state_t;
    state_t state, state_next;

    // owner: 0=idma, 1=wdma; 仅在 S_CMD/S_DATA/S_STS 有效
    logic owner, owner_next;

    // 派生
    logic cmd_fire, data_fire, tlast_fire, sts_fire;
    assign cmd_fire   = mm2s_cmd_tvalid  && mm2s_cmd_tready;
    assign data_fire  = mm2s_data_tvalid && mm2s_data_tready;
    assign tlast_fire = data_fire && mm2s_data_tlast;
    assign sts_fire   = mm2s_sts_tvalid  && mm2s_sts_tready;

    // 选 owner: idma 优先
    always_comb begin
        if      (idma_cmd_tvalid) owner_next = 1'b0;
        else if (wdma_cmd_tvalid) owner_next = 1'b1;
        else                      owner_next = owner;
    end

    // FSM 转移
    always_comb begin
        state_next = state;
        case (state)
            S_IDLE : if (idma_cmd_tvalid || wdma_cmd_tvalid) state_next = S_CMD;
            S_CMD  : if (cmd_fire)                            state_next = S_DATA;
            S_DATA : if (tlast_fire)                          state_next = S_STS;
            S_STS  : if (sts_fire)                            state_next = S_IDLE;
            default:                                          state_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= state_next;
    end

    always_ff @(posedge clk) begin
        if      (!rst_n)            owner <= 1'b0;
        else if (state == S_IDLE)   owner <= owner_next;
        else                        owner <= owner;
    end

    // =========================================================================
    // CMD 路由 (S_CMD: 选 owner 的 cmd)
    // =========================================================================
    assign mm2s_cmd_tdata  = (owner == 1'b0) ? idma_cmd_tdata  : wdma_cmd_tdata;
    assign mm2s_cmd_tvalid = (state == S_CMD) &&
                             ((owner == 1'b0) ? idma_cmd_tvalid : wdma_cmd_tvalid);
    assign idma_cmd_tready = (state == S_CMD) && (owner == 1'b0) && mm2s_cmd_tready;
    assign wdma_cmd_tready = (state == S_CMD) && (owner == 1'b1) && mm2s_cmd_tready;

    // =========================================================================
    // DATA 路由 (S_DATA: data 流给 owner)
    // =========================================================================
    assign idma_data_tvalid = (state == S_DATA) && (owner == 1'b0) && mm2s_data_tvalid;
    assign idma_data_tdata  = mm2s_data_tdata;
    assign idma_data_tkeep  = mm2s_data_tkeep;
    assign idma_data_tlast  = mm2s_data_tlast;

    assign wdma_data_tvalid = (state == S_DATA) && (owner == 1'b1) && mm2s_data_tvalid;
    assign wdma_data_tdata  = mm2s_data_tdata;
    assign wdma_data_tkeep  = mm2s_data_tkeep;
    assign wdma_data_tlast  = mm2s_data_tlast;

    assign mm2s_data_tready = (state == S_DATA) &&
                              ((owner == 1'b0) ? idma_data_tready : wdma_data_tready);

    // =========================================================================
    // STS 路由 (S_STS: sts 给 owner)
    // =========================================================================
    assign idma_sts_tvalid = (state == S_STS) && (owner == 1'b0) && mm2s_sts_tvalid;
    assign idma_sts_tdata  = mm2s_sts_tdata;

    assign wdma_sts_tvalid = (state == S_STS) && (owner == 1'b1) && mm2s_sts_tvalid;
    assign wdma_sts_tdata  = mm2s_sts_tdata;

    assign mm2s_sts_tready = (state == S_STS) &&
                             ((owner == 1'b0) ? idma_sts_tready : wdma_sts_tready);

endmodule
