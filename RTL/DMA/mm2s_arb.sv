`timescale 1ns/1ps

// =============================================================================
// mm2s_arb.sv  --  AXI DataMover MM2S 通道仲裁器 (idma_ctrl + wdma_ctrl 共用)
//
// Pipelined 版本: 利用 DataMover 内部 cmd FIFO (默认深度 4) 让两路上游可以多
// cmd in-flight, 提高 K=1 ds 这类 "短 row + per-cmd pipeline depth 暴露" 场景
// 的吞吐. 旧的 strict-serial 版本每发一条 cmd 要跑完 cmd→data→sts 才能发下一
// 条, 浪费了 DataMover 的流水能力.
//
// 仲裁:
//   - 当 idma + wdma 同时 cmd_tvalid, idma 优先 (流式负载更敏感; wdma 是一次
//     性预加载, 等几拍无所谓)
//   - cmd_tready 由下游 axi_dm 自动反压 (cmd FIFO 满会拉低), 加上本模块的
//     owner FIFO 满也反压, 永不溢出
//
// data / sts demux: 用 2 个独立 owner FIFO 跟踪 in-flight cmd 的 owner 顺序.
// DataMover 严格按 cmd 顺序输出 data + sts, 所以两个 FIFO 都按 cmd 顺序排队,
// 但消费节奏不同:
//   - data_owner_fifo: cmd_fire 时 push owner; data tlast_fire 时 pop
//                       FIFO head = 当前 data 流路由到的 owner
//   - sts_owner_fifo:  cmd_fire 时 push owner; sts_fire 时 pop
//                       FIFO head = 当前 sts 路由到的 owner
// 两者 push 同步 (cmd_fire 同拍 push 两个), pop 各自独立.
//
// FIFO 深度 8: 容纳 DataMover cmd FIFO 4 + 内部流水 ≤4 余量, 永不会成为瓶颈.
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

    localparam int PTR_W = $clog2(OFIFO_DEPTH);
    localparam int CNT_W = $clog2(OFIFO_DEPTH + 1);

    // =========================================================================
    // CMD 仲裁: idma 优先, 同时 cmd_tvalid 时选 idma
    // =========================================================================
    logic cmd_owner;     // 0=idma, 1=wdma; combinational 选择
    assign cmd_owner = idma_cmd_tvalid ? 1'b0 : 1'b1;

    logic any_cmd_tvalid;
    assign any_cmd_tvalid = idma_cmd_tvalid | wdma_cmd_tvalid;

    // owner FIFO 满时反压 cmd 发射
    logic data_full, sts_full;

    assign mm2s_cmd_tdata  = (cmd_owner == 1'b0) ? idma_cmd_tdata : wdma_cmd_tdata;
    assign mm2s_cmd_tvalid = any_cmd_tvalid && !data_full && !sts_full;

    logic cmd_fire;
    assign cmd_fire = mm2s_cmd_tvalid && mm2s_cmd_tready;

    assign idma_cmd_tready = idma_cmd_tvalid && mm2s_cmd_tready && !data_full && !sts_full;
    assign wdma_cmd_tready = !idma_cmd_tvalid && wdma_cmd_tvalid && mm2s_cmd_tready && !data_full && !sts_full;

    // =========================================================================
    // owner FIFO × 2 (data / sts 各一个)
    //   两者同时 push (cmd_fire 时), pop 各自异步:
    //     data_fifo  pop on data_tlast_fire (= 当前 cmd 数据全部送完)
    //     sts_fifo   pop on sts_fire        (= 当前 cmd commit 完)
    //   sts 总在 data tlast 之后, 所以 sts_fifo 的占用 ≥ data_fifo (但深度同).
    // =========================================================================
    logic [OFIFO_DEPTH-1:0] data_mem, sts_mem;
    logic [PTR_W-1:0]       data_wr, data_rd;
    logic [PTR_W-1:0]       sts_wr,  sts_rd;
    logic [CNT_W-1:0]       data_cnt, sts_cnt;
    logic                   data_empty, sts_empty;
    logic                   data_head, sts_head;

    assign data_empty = (data_cnt == '0);
    assign data_full  = (data_cnt == CNT_W'(OFIFO_DEPTH));
    assign data_head  = data_mem[data_rd];

    assign sts_empty  = (sts_cnt == '0);
    assign sts_full   = (sts_cnt == CNT_W'(OFIFO_DEPTH));
    assign sts_head   = sts_mem[sts_rd];

    logic data_tlast_fire, sts_fire;
    assign data_tlast_fire = mm2s_data_tvalid && mm2s_data_tready && mm2s_data_tlast;
    assign sts_fire        = mm2s_sts_tvalid  && mm2s_sts_tready;

    // ---- data_fifo ----
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

    // ---- sts_fifo ----
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
    // DATA 路由: mm2s_data → data_head 选中的 owner
    // =========================================================================
    assign idma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 1'b0);
    assign idma_data_tdata  = mm2s_data_tdata;
    assign idma_data_tkeep  = mm2s_data_tkeep;
    assign idma_data_tlast  = mm2s_data_tlast;

    assign wdma_data_tvalid = mm2s_data_tvalid && !data_empty && (data_head == 1'b1);
    assign wdma_data_tdata  = mm2s_data_tdata;
    assign wdma_data_tkeep  = mm2s_data_tkeep;
    assign wdma_data_tlast  = mm2s_data_tlast;

    assign mm2s_data_tready = !data_empty &&
                              ((data_head == 1'b0) ? idma_data_tready : wdma_data_tready);

    // =========================================================================
    // STS 路由: mm2s_sts → sts_head 选中的 owner
    // =========================================================================
    assign idma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 1'b0);
    assign idma_sts_tdata  = mm2s_sts_tdata;

    assign wdma_sts_tvalid = mm2s_sts_tvalid && !sts_empty && (sts_head == 1'b1);
    assign wdma_sts_tdata  = mm2s_sts_tdata;

    assign mm2s_sts_tready = !sts_empty &&
                             ((sts_head == 1'b0) ? idma_sts_tready : wdma_sts_tready);

endmodule
