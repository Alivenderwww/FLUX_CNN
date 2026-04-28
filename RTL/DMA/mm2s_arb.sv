`timescale 1ns/1ps

// =============================================================================
// mm2s_arb.sv  --  AXI DataMover MM2S 通道仲裁器 (3 路: idma + wdma + rdma)
//
// Pipelined: 利用 DataMover 内部 cmd FIFO 让多 cmd in-flight. cmd 严格按入序
// 处理, 因此 data / sts 的目的端 (owner) 只需用 FIFO 跟踪 cmd 入序即可.
//
// 仲裁优先级 (高→低): idma > rdma > wdma
//   - idma: 流式输入, 实时性最敏感
//   - rdma: 残差/bias 加载, layer 启动期一次性 (只有 1 条 cmd)
//   - wdma: 权重加载, layer 启动期一次性
// rdma + wdma 都是 batch, 但 rdma 需要在 layer compute 开始前完成 (bias_rf
// 依赖), 略优先于 wdma.
//
// data / sts 各一个 owner FIFO (深度 8). owner 编码: 2'd0=idma, 2'd1=wdma,
// 2'd2=rdma. push on cmd_fire, pop 各异步.
// =============================================================================

module mm2s_arb #(
    parameter int DATA_W      = 128,
    parameter int OFIFO_DEPTH = 8
)(
    input  logic                    clk,
    input  logic                    rst_n,

    // ---- 端口 0: idma_ctrl ----
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

    // ---- 端口 1: wdma_ctrl ----
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

    // ---- 端口 2: rdma_ctrl ----
    input  logic                    rdma_cmd_tvalid,
    output logic                    rdma_cmd_tready,
    input  logic [71:0]             rdma_cmd_tdata,
    output logic                    rdma_data_tvalid,
    input  logic                    rdma_data_tready,
    output logic [DATA_W-1:0]       rdma_data_tdata,
    output logic [DATA_W/8-1:0]     rdma_data_tkeep,
    output logic                    rdma_data_tlast,
    output logic                    rdma_sts_tvalid,
    input  logic                    rdma_sts_tready,
    output logic [7:0]              rdma_sts_tdata,

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

    localparam int OWN_W = 2;     // 0=idma, 1=wdma, 2=rdma
    localparam int PTR_W = $clog2(OFIFO_DEPTH);
    localparam int CNT_W = $clog2(OFIFO_DEPTH + 1);

    // =========================================================================
    // CMD 仲裁 (priority): idma > rdma > wdma
    // =========================================================================
    logic [OWN_W-1:0] cmd_owner;
    always_comb begin
        if      (idma_cmd_tvalid) cmd_owner = 2'd0;
        else if (rdma_cmd_tvalid) cmd_owner = 2'd2;
        else                       cmd_owner = 2'd1;     // wdma 默认
    end

    logic any_cmd_tvalid;
    assign any_cmd_tvalid = idma_cmd_tvalid | wdma_cmd_tvalid | rdma_cmd_tvalid;

    logic data_full, sts_full;

    always_comb begin
        case (cmd_owner)
            2'd0   : mm2s_cmd_tdata = idma_cmd_tdata;
            2'd1   : mm2s_cmd_tdata = wdma_cmd_tdata;
            2'd2   : mm2s_cmd_tdata = rdma_cmd_tdata;
            default: mm2s_cmd_tdata = idma_cmd_tdata;
        endcase
    end
    assign mm2s_cmd_tvalid = any_cmd_tvalid && !data_full && !sts_full;

    logic cmd_fire;
    assign cmd_fire = mm2s_cmd_tvalid && mm2s_cmd_tready;

    assign idma_cmd_tready = (cmd_owner == 2'd0) && idma_cmd_tvalid && mm2s_cmd_tready && !data_full && !sts_full;
    assign wdma_cmd_tready = (cmd_owner == 2'd1) && wdma_cmd_tvalid && mm2s_cmd_tready && !data_full && !sts_full;
    assign rdma_cmd_tready = (cmd_owner == 2'd2) && rdma_cmd_tvalid && mm2s_cmd_tready && !data_full && !sts_full;

    // =========================================================================
    // owner FIFO × 2 (data / sts), each depth = OFIFO_DEPTH, width = OWN_W
    // =========================================================================
    logic [OWN_W-1:0]  data_mem [0:OFIFO_DEPTH-1];
    logic [OWN_W-1:0]  sts_mem  [0:OFIFO_DEPTH-1];
    logic [PTR_W-1:0]  data_wr, data_rd;
    logic [PTR_W-1:0]  sts_wr,  sts_rd;
    logic [CNT_W-1:0]  data_cnt, sts_cnt;
    logic              data_empty, sts_empty;
    logic [OWN_W-1:0]  data_head, sts_head;

    assign data_empty = (data_cnt == '0);
    assign data_full  = (data_cnt == CNT_W'(OFIFO_DEPTH));
    assign data_head  = data_mem[data_rd];

    assign sts_empty  = (sts_cnt == '0);
    assign sts_full   = (sts_cnt == CNT_W'(OFIFO_DEPTH));
    assign sts_head   = sts_mem[sts_rd];

    logic data_tlast_fire, sts_fire;
    assign data_tlast_fire = mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast;
    assign sts_fire        = mm2s_sts_tvalid  && mm2s_sts_tready;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            data_wr  <= '0;
            data_rd  <= '0;
            data_cnt <= '0;
        end else begin
            if (cmd_fire) begin
                data_mem[data_wr] <= cmd_owner;
                data_wr           <= data_wr + 1'b1;
            end
            if (data_tlast_fire) data_rd <= data_rd + 1'b1;
            case ({cmd_fire, data_tlast_fire})
                2'b10  : data_cnt <= data_cnt + 1'b1;
                2'b01  : data_cnt <= data_cnt - 1'b1;
                default: data_cnt <= data_cnt;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sts_wr  <= '0;
            sts_rd  <= '0;
            sts_cnt <= '0;
        end else begin
            if (cmd_fire) begin
                sts_mem[sts_wr] <= cmd_owner;
                sts_wr          <= sts_wr + 1'b1;
            end
            if (sts_fire) sts_rd <= sts_rd + 1'b1;
            case ({cmd_fire, sts_fire})
                2'b10  : sts_cnt <= sts_cnt + 1'b1;
                2'b01  : sts_cnt <= sts_cnt - 1'b1;
                default: sts_cnt <= sts_cnt;
            endcase
        end
    end

    // =========================================================================
    // DATA demux (按 data_head)
    // =========================================================================
    assign idma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd0);
    assign idma_data_tdata  = mm2s_data_tdata;
    assign idma_data_tkeep  = mm2s_data_tkeep;
    assign idma_data_tlast  = mm2s_data_tlast;

    assign wdma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd1);
    assign wdma_data_tdata  = mm2s_data_tdata;
    assign wdma_data_tkeep  = mm2s_data_tkeep;
    assign wdma_data_tlast  = mm2s_data_tlast;

    assign rdma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 2'd2);
    assign rdma_data_tdata  = mm2s_data_tdata;
    assign rdma_data_tkeep  = mm2s_data_tkeep;
    assign rdma_data_tlast  = mm2s_data_tlast;

    always_comb begin
        case (data_head)
            2'd0   : mm2s_data_tready = !data_empty && idma_data_tready;
            2'd1   : mm2s_data_tready = !data_empty && wdma_data_tready;
            2'd2   : mm2s_data_tready = !data_empty && rdma_data_tready;
            default: mm2s_data_tready = 1'b0;
        endcase
    end

    // =========================================================================
    // STS demux (按 sts_head)
    // =========================================================================
    assign idma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd0);
    assign idma_sts_tdata  = mm2s_sts_tdata;

    assign wdma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd1);
    assign wdma_sts_tdata  = mm2s_sts_tdata;

    assign rdma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 2'd2);
    assign rdma_sts_tdata  = mm2s_sts_tdata;

    always_comb begin
        case (sts_head)
            2'd0   : mm2s_sts_tready = !sts_empty && idma_sts_tready;
            2'd1   : mm2s_sts_tready = !sts_empty && wdma_sts_tready;
            2'd2   : mm2s_sts_tready = !sts_empty && rdma_sts_tready;
            default: mm2s_sts_tready = 1'b0;
        endcase
    end

endmodule
