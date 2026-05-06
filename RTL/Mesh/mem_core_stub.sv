`timescale 1ns/1ps

// =============================================================================
// mem_core_stub.sv  --  简化 Mem Core 模型 (mesh PoC sim)
//
// [Step F.4 优化] 内置 desc engine:
//   host 把 desc list preload 到 mem.ddr_mem 某区域 → 写 DESC_LIST_ADDR / COUNT
//   + start_pulse → mem desc engine 自动从 ddr_mem 拉 desc, 顺序 trigger
//   axis_packet_tx 发包. 4 个 mem 真并行 (host CSR 总线只写 3 个寄存器/层).
//
// 行为:
//   1. 接 mesh LOCAL port (1 AXIS slave + 1 AXIS master)
//   2. 内部 "DDR mock" array (ddr_mem) 既存 IFB 数据, 又存 desc list
//   3. axis_packet_rx → 写 ddr_mem (收 packet, e.g. ConvCore 出 OFM)
//   4. desc engine + axis_packet_tx → 从 ddr_mem 读 + 发 packet
//
// AXI-Lite CSR map (12-bit addr, 32-bit data):
//   0x000  DESC_LIST_ADDR   [31:0]   desc list 在 ddr_mem 内 word offset
//   0x004  DESC_COUNT       [15:0]   desc 个数
//   0x008  CTRL             W: bit0=1 → start_pulse 拉一拍, 自动清 done_sticky
//   0x00C  STATUS  (RO)     [0]=engine_busy, [1]=engine_done_sticky,
//                            [2]=rx_pkt_done_sticky
//
// Desc 二进制格式 (跟 mesh_cmd.py 同步, 16 byte = 1 ddr_mem word):
//   bit [31:0]    ddr_addr_w     mem.ddr_mem 内 word offset (读起点)
//   bit [47:32]   burst_len      packet beat 数
//   bit [79:48]   sram_offset_w  packet header 里的 sram offset (32-bit, 实际 20-bit 用)
//   bit [87:80]   tdest          8-bit
//   bit [91:88]   opcode         4-bit
//   bit [127:92]  reserved       36-bit, 0
//
// 真 Mem Core 替代说明:
//   stub 的 ddr_mem array 替换为 axi_dm IP + 真 DDR
//   stub 的 desc engine 替换为 mem 内 真实 axi master + axi_dm 拉 desc
// =============================================================================

module mem_core_stub #(
    parameter int DATA_W   = 128,
    parameter int DEST_W   = 8,
    parameter int ADDR_W   = 20,
    parameter int DDR_DEPTH = 65536,
    parameter int CSR_ADDR_W = 12,
    parameter int CSR_DATA_W = 32
)(
    input  logic                clk,
    input  logic                rst_n,

    // ---- mesh LOCAL port (AXIS) ----
    input  logic                s_axis_tvalid,
    output logic                s_axis_tready,
    input  logic [DATA_W-1:0]   s_axis_tdata,
    input  logic                s_axis_tlast,
    input  logic [DEST_W-1:0]   s_axis_tdest,

    output logic                m_axis_tvalid,
    input  logic                m_axis_tready,
    output logic [DATA_W-1:0]   m_axis_tdata,
    output logic                m_axis_tlast,
    output logic [DEST_W-1:0]   m_axis_tdest,

    // ---- AXI-Lite CSR Slave (host 配置 desc engine) ----
    input  logic [CSR_ADDR_W-1:0]    csr_awaddr,
    input  logic                     csr_awvalid,
    output logic                     csr_awready,
    input  logic [CSR_DATA_W-1:0]    csr_wdata,
    input  logic [CSR_DATA_W/8-1:0]  csr_wstrb,
    input  logic                     csr_wvalid,
    output logic                     csr_wready,
    output logic [1:0]               csr_bresp,
    output logic                     csr_bvalid,
    input  logic                     csr_bready,
    input  logic [CSR_ADDR_W-1:0]    csr_araddr,
    input  logic                     csr_arvalid,
    output logic                     csr_arready,
    output logic [CSR_DATA_W-1:0]    csr_rdata,
    output logic [1:0]               csr_rresp,
    output logic                     csr_rvalid,
    input  logic                     csr_rready,

    // ---- 调试观察 (RX 最近一包 header) ----
    output logic [3:0]          last_rx_opcode,
    output logic [15:0]         last_rx_burst_len,
    output logic [ADDR_W-1:0]   last_rx_addr,
    output logic                rx_pkt_done
);

    // CSR addr 常量
    localparam [CSR_ADDR_W-1:0] ADDR_DESC_LIST_ADDR = 12'h000;
    localparam [CSR_ADDR_W-1:0] ADDR_DESC_COUNT     = 12'h004;
    localparam [CSR_ADDR_W-1:0] ADDR_CTRL           = 12'h008;
    localparam [CSR_ADDR_W-1:0] ADDR_STATUS         = 12'h00C;

    // ----- DDR mock array -----
    logic [DATA_W-1:0]   ddr_mem [0:DDR_DEPTH-1];

    // =========================================================================
    // AXI-Lite CSR slave → reg_w_*/reg_r_* 寄存器 bank
    // =========================================================================
    logic                     reg_w_en;
    logic [CSR_ADDR_W-1:0]    reg_w_addr;
    logic [CSR_DATA_W-1:0]    reg_w_data;
    logic [CSR_DATA_W/8-1:0]  reg_w_strb;
    logic [CSR_ADDR_W-1:0]    reg_r_addr;
    logic [CSR_DATA_W-1:0]    reg_r_data;

    axi_lite_csr #(.ADDR_W(CSR_ADDR_W), .DATA_W(CSR_DATA_W)) u_csr (
        .clk(clk), .rstn(rst_n),
        .AWADDR(csr_awaddr), .AWVALID(csr_awvalid), .AWREADY(csr_awready),
        .WDATA(csr_wdata),   .WSTRB(csr_wstrb),
        .WVALID(csr_wvalid), .WREADY(csr_wready),
        .BRESP(csr_bresp),   .BVALID(csr_bvalid), .BREADY(csr_bready),
        .ARADDR(csr_araddr), .ARVALID(csr_arvalid), .ARREADY(csr_arready),
        .RDATA(csr_rdata),   .RRESP(csr_rresp),
        .RVALID(csr_rvalid), .RREADY(csr_rready),
        .reg_w_en(reg_w_en), .reg_w_addr(reg_w_addr),
        .reg_w_data(reg_w_data), .reg_w_strb(reg_w_strb),
        .reg_r_addr(reg_r_addr), .reg_r_data(reg_r_data)
    );

    // =========================================================================
    // CSR 寄存器
    // =========================================================================
    logic [31:0]      r_desc_list_addr;
    logic [15:0]      r_desc_count;
    logic             start_pulse;
    logic             engine_busy;
    logic             engine_done_pulse;
    logic             r_engine_done_sticky;
    logic             r_rx_pkt_done_sticky;

    assign start_pulse = reg_w_en && (reg_w_addr == ADDR_CTRL) && reg_w_data[0];

    always_ff @(posedge clk) begin
        if (reg_w_en) begin
            case (reg_w_addr)
                ADDR_DESC_LIST_ADDR : r_desc_list_addr <= reg_w_data;
                ADDR_DESC_COUNT     : r_desc_count     <= reg_w_data[15:0];
                default: ;
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n)                   r_engine_done_sticky <= 1'b0;
        else if (start_pulse)         r_engine_done_sticky <= 1'b0;
        else if (engine_done_pulse)   r_engine_done_sticky <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (!rst_n)              r_rx_pkt_done_sticky <= 1'b0;
        else if (rx_pkt_done)    r_rx_pkt_done_sticky <= 1'b1;
    end

    always_comb begin
        case (reg_r_addr)
            ADDR_DESC_LIST_ADDR : reg_r_data = r_desc_list_addr;
            ADDR_DESC_COUNT     : reg_r_data = {16'd0, r_desc_count};
            ADDR_STATUS         : reg_r_data = {29'd0, r_rx_pkt_done_sticky,
                                                 r_engine_done_sticky, engine_busy};
            default             : reg_r_data = 32'd0;
        endcase
    end

    // =========================================================================
    // Desc Engine FSM
    //   S_IDLE   → start_pulse → S_FETCH
    //   S_FETCH  : 1 拍读 ddr_mem[desc_addr] (寄存器输出, 下拍可用)
    //   S_DECODE : 1 拍 latch 解 desc 字段, → S_ISSUE
    //   S_ISSUE  : trigger axis_packet_tx (start 拉 1 拍), → S_WAIT
    //   S_WAIT   : 等 tx done_pulse, desc_idx++, 是否最后一条? → S_DONE / S_FETCH
    //   S_DONE   : engine_done_pulse 拉 1 拍 → S_IDLE
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE   = 3'd0,
        S_FETCH  = 3'd1,
        S_DECODE = 3'd2,
        S_ISSUE  = 3'd3,
        S_WAIT   = 3'd4,
        S_DONE   = 3'd5
    } eng_state_t;
    eng_state_t eng_st;

    logic [31:0]      r_desc_idx;
    logic [DATA_W-1:0] r_desc_word;       // 当前 desc 的 raw 16 byte
    logic [31:0]      r_cmd_ddr_addr;
    logic [15:0]      r_cmd_burst_len;
    logic [ADDR_W-1:0] r_cmd_sram_offset;
    logic [3:0]       r_cmd_opcode;
    logic [DEST_W-1:0]r_cmd_tdest;
    logic             tx_start_pulse;
    logic             tx_busy;
    logic             tx_done_pulse;

    assign engine_busy = (eng_st != S_IDLE);

    // ddr_mem 读: desc 拉 + tx body 数据拉 共享 (mux)
    logic                desc_re;
    logic [ADDR_W-1:0]   desc_raddr;
    logic [DATA_W-1:0]   desc_rdata_lat;
    logic                tx_sram_re;
    logic [ADDR_W-1:0]   tx_sram_raddr;
    logic [DATA_W-1:0]   tx_sram_rdata_lat;

    // ddr_mem read port mux (3 家共享):
    //   desc_re      = push 模式 desc engine fetch desc
    //   tx_sram_re   = push 模式 axis_packet_tx 拉 body 数据
    //   rrq_re       = NUMA 模式 axis_packet_read 拉 RESP body 数据
    // 三家不会并发 (push 跟 NUMA 不同时), 简单 OR + priority mux
    logic                ddr_re;
    logic [ADDR_W-1:0]   ddr_raddr;
    logic [DATA_W-1:0]   ddr_rdata;
    // NUMA path 信号 (axis_packet_read 实例的输出, 这里前向声明给 ddr mux 用)
    logic                rrq_re;
    logic [ADDR_W-1:0]   rrq_raddr;
    logic [DATA_W-1:0]   rrq_rdata;

    assign ddr_re    = desc_re || tx_sram_re || rrq_re;
    assign ddr_raddr = desc_re   ? desc_raddr     :
                       rrq_re    ? rrq_raddr      :
                                   tx_sram_raddr;
    always_ff @(posedge clk) if (ddr_re) ddr_rdata <= ddr_mem[ddr_raddr];

    // axis_packet_read 看自己的 ddr_rdata (上拍 rrq_re 拉的)
    assign rrq_rdata = ddr_rdata;

    // desc/tx 各自从 ddr_rdata 锁存最新值 (上一拍 re 拉的)
    logic                r_desc_re_d1;
    logic                r_tx_sram_re_d1;
    always_ff @(posedge clk) begin
        r_desc_re_d1    <= desc_re;
        r_tx_sram_re_d1 <= tx_sram_re;
    end

    // FSM 主体 (3 段式)
    eng_state_t eng_st_next;
    always_comb begin
        eng_st_next = eng_st;
        case (eng_st)
            S_IDLE   : if (start_pulse && r_desc_count > 0) eng_st_next = S_FETCH;
            S_FETCH  : eng_st_next = S_DECODE;          // 1 拍读 ddr_mem
            S_DECODE : eng_st_next = S_ISSUE;           // 1 拍 latch
            S_ISSUE  : eng_st_next = S_WAIT;            // tx_start_pulse 1 拍
            S_WAIT   : if (tx_done_pulse) begin
                if (r_desc_idx + 32'd1 >= {16'd0, r_desc_count}) eng_st_next = S_DONE;
                else                                              eng_st_next = S_FETCH;
            end
            S_DONE   : eng_st_next = S_IDLE;
            default  : eng_st_next = S_IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) eng_st <= S_IDLE;
        else        eng_st <= eng_st_next;
    end

    // desc_idx (从 0 开始)
    always_ff @(posedge clk) begin
        if      (!rst_n)             r_desc_idx <= '0;
        else if (start_pulse)        r_desc_idx <= '0;
        else if (eng_st == S_WAIT && tx_done_pulse) r_desc_idx <= r_desc_idx + 32'd1;
    end

    // desc fetch read pulse + addr (S_FETCH 拍发 ddr re, 1 拍后 ddr_rdata 有效)
    assign desc_re    = (eng_st == S_FETCH);
    assign desc_raddr = r_desc_list_addr[ADDR_W-1:0] + r_desc_idx[ADDR_W-1:0];

    // S_DECODE 时 ddr_rdata 是上拍 re 的结果 (r_desc_re_d1=1), latch 字段
    always_ff @(posedge clk) begin
        if (eng_st == S_DECODE) begin
            r_desc_word       <= ddr_rdata;
            r_cmd_ddr_addr    <= ddr_rdata[31:0];
            r_cmd_burst_len   <= ddr_rdata[47:32];
            r_cmd_sram_offset <= ddr_rdata[48 +: ADDR_W];
            r_cmd_tdest       <= ddr_rdata[80 +: DEST_W];
            r_cmd_opcode      <= ddr_rdata[91:88];
        end
    end

    // S_ISSUE 拉 tx start
    assign tx_start_pulse = (eng_st == S_ISSUE);
    assign engine_done_pulse = (eng_st == S_DONE);

    // =========================================================================
    // s_axis demux: 按 packet 第 1 flit 的 opcode 分流
    //   opcode 0x0 / 0x5 (WRITE / WRITE_DDR_OFB) → axis_packet_rx (写 ddr_mem)
    //   opcode 0x1 (READ_REQ, NUMA)              → axis_packet_read (读 ddr_mem 发 RESP)
    //   其他: 丢弃
    //
    // routing 状态机锁存当前 packet 路由方向 (mesh wormhole 保证 packet 内连续)
    // =========================================================================
    typedef enum logic [1:0] { ROUTE_IDLE, ROUTE_WRITE, ROUTE_READ_REQ } route_t;
    route_t routing;

    logic [3:0] hdr_opcode;
    assign hdr_opcode = s_axis_tdata[127:124];

    // axis_packet_rx 接口
    logic                rx_tvalid;
    logic                rx_tready;
    // axis_packet_read 接口
    logic                rrq_tvalid;
    logic                rrq_tready;

    always_comb begin
        rx_tvalid     = 1'b0;
        rrq_tvalid    = 1'b0;
        s_axis_tready = 1'b0;
        case (routing)
            ROUTE_IDLE: begin
                // 第 1 flit: 看 opcode 决定路由
                if (hdr_opcode == 4'h1) begin
                    rrq_tvalid    = s_axis_tvalid;
                    s_axis_tready = rrq_tready;
                end else begin
                    rx_tvalid     = s_axis_tvalid;
                    s_axis_tready = rx_tready;
                end
            end
            ROUTE_WRITE: begin
                rx_tvalid     = s_axis_tvalid;
                s_axis_tready = rx_tready;
            end
            ROUTE_READ_REQ: begin
                rrq_tvalid    = s_axis_tvalid;
                s_axis_tready = rrq_tready;
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (!rst_n) routing <= ROUTE_IDLE;
        else if (s_axis_tvalid && s_axis_tready) begin
            if (routing == ROUTE_IDLE) begin
                if (hdr_opcode == 4'h1) routing <= s_axis_tlast ? ROUTE_IDLE : ROUTE_READ_REQ;
                else                    routing <= s_axis_tlast ? ROUTE_IDLE : ROUTE_WRITE;
            end else if (s_axis_tlast) begin
                routing <= ROUTE_IDLE;
            end
        end
    end

    // =========================================================================
    // axis_packet_rx → 写 DDR mock (WRITE / WRITE_DDR_OFB packet)
    // =========================================================================
    logic                rx_we;
    logic [3:0]          rx_target;
    logic [ADDR_W-1:0]   rx_waddr;
    logic [DATA_W-1:0]   rx_wdata;

    axis_packet_rx #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_rx (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(rx_tvalid),     .s_axis_tready(rx_tready),
        .s_axis_tlast(s_axis_tlast),   .s_axis_tdata(s_axis_tdata),
        .s_axis_tdest(s_axis_tdest),
        .sram_we(rx_we), .sram_target(rx_target),
        .sram_waddr(rx_waddr), .sram_wdata(rx_wdata),
        .packet_done(rx_pkt_done),
        .last_opcode(last_rx_opcode),
        .last_burst_len(last_rx_burst_len),
        .last_addr_offset(last_rx_addr)
    );

    always_ff @(posedge clk) begin
        if (rx_we && rx_waddr < DDR_DEPTH)
            ddr_mem[rx_waddr] <= rx_wdata;
    end

    // =========================================================================
    // axis_packet_read → 收 READ_REQ packet, 读 ddr_mem, 发 READ_RESP packet (NUMA)
    //   (rrq_re/raddr/rdata 已在前面 ddr_mem read mux 区域声明, 这里直接用)
    // =========================================================================
    // RESP packet 出口 (mux 跟 desc engine push 出口, 见后)
    logic                resp_tvalid;
    logic                resp_tready;
    logic [DATA_W-1:0]   resp_tdata;
    logic                resp_tlast;
    logic [DEST_W-1:0]   resp_tdest;

    axis_packet_read #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_read (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(rrq_tvalid),    .s_axis_tready(rrq_tready),
        .s_axis_tdata(s_axis_tdata),   .s_axis_tlast(s_axis_tlast),
        .s_axis_tdest(s_axis_tdest),
        .ddr_re(rrq_re), .ddr_raddr(rrq_raddr), .ddr_rdata(rrq_rdata),
        .m_axis_tvalid(resp_tvalid),   .m_axis_tready(resp_tready),
        .m_axis_tdata(resp_tdata),     .m_axis_tlast(resp_tlast),
        .m_axis_tdest(resp_tdest)
    );

    // =========================================================================
    // axis_packet_tx → 从 DDR mock 读 + 发 packet (engine 驱动)
    // =========================================================================
    // tx_sram_rdata 用 ddr_rdata (上拍 tx_sram_re 拉的)
    assign tx_sram_rdata_lat = ddr_rdata;

    // push 模式 axis_packet_tx 出口 (内部信号, 后面跟 NUMA RESP mux)
    logic                push_tx_tvalid;
    logic                push_tx_tready;
    logic [DATA_W-1:0]   push_tx_tdata;
    logic                push_tx_tlast;
    logic [DEST_W-1:0]   push_tx_tdest;

    axis_packet_tx #(
        .DATA_W(DATA_W), .DEST_W(DEST_W), .ADDR_W(ADDR_W)
    ) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tx_start_pulse),
        .cfg_opcode(r_cmd_opcode),
        .cfg_sram_addr(r_cmd_sram_offset),
        .cfg_burst_len(r_cmd_burst_len),
        .cfg_tdest(r_cmd_tdest),
        .busy(tx_busy), .done(tx_done_pulse),
        .sram_re(tx_sram_re), .sram_raddr(tx_sram_raddr), .sram_rdata(tx_sram_rdata_lat),
        .m_axis_tvalid(push_tx_tvalid), .m_axis_tready(push_tx_tready),
        .m_axis_tdata(push_tx_tdata),   .m_axis_tlast(push_tx_tlast),
        .m_axis_tdest(push_tx_tdest)
    );

    // =========================================================================
    // m_axis 出口 mux: push tx (desc engine 推 IFB) vs NUMA RESP (axis_packet_read)
    //   两路不会并发 (push 跟 NUMA 模式互斥, 见 ddr 读端口注释)
    //   priority: push tx 优先 (legacy 兼容); 实际 NUMA 模式下 push 不启
    // 用 packet-locked mux 避免 wormhole 中途切换
    // =========================================================================
    typedef enum logic [1:0] { TX_IDLE, TX_PUSH, TX_RESP } tx_route_t;
    tx_route_t tx_route;

    always_ff @(posedge clk) begin
        if (!rst_n) tx_route <= TX_IDLE;
        else begin
            case (tx_route)
                TX_IDLE: begin
                    if      (push_tx_tvalid) tx_route <= push_tx_tlast ? TX_IDLE : TX_PUSH;
                    else if (resp_tvalid)    tx_route <= resp_tlast    ? TX_IDLE : TX_RESP;
                end
                TX_PUSH: if (push_tx_tvalid && push_tx_tready && push_tx_tlast) tx_route <= TX_IDLE;
                TX_RESP: if (resp_tvalid && resp_tready && resp_tlast)          tx_route <= TX_IDLE;
            endcase
        end
    end

    always_comb begin
        m_axis_tvalid  = 1'b0;
        m_axis_tdata   = '0;
        m_axis_tlast   = 1'b0;
        m_axis_tdest   = '0;
        push_tx_tready = 1'b0;
        resp_tready    = 1'b0;
        case (tx_route)
            TX_IDLE: begin
                if (push_tx_tvalid) begin
                    m_axis_tvalid  = push_tx_tvalid;
                    m_axis_tdata   = push_tx_tdata;
                    m_axis_tlast   = push_tx_tlast;
                    m_axis_tdest   = push_tx_tdest;
                    push_tx_tready = m_axis_tready;
                end else if (resp_tvalid) begin
                    m_axis_tvalid  = resp_tvalid;
                    m_axis_tdata   = resp_tdata;
                    m_axis_tlast   = resp_tlast;
                    m_axis_tdest   = resp_tdest;
                    resp_tready    = m_axis_tready;
                end
            end
            TX_PUSH: begin
                m_axis_tvalid  = push_tx_tvalid;
                m_axis_tdata   = push_tx_tdata;
                m_axis_tlast   = push_tx_tlast;
                m_axis_tdest   = push_tx_tdest;
                push_tx_tready = m_axis_tready;
            end
            TX_RESP: begin
                m_axis_tvalid  = resp_tvalid;
                m_axis_tdata   = resp_tdata;
                m_axis_tlast   = resp_tlast;
                m_axis_tdest   = resp_tdest;
                resp_tready    = m_axis_tready;
            end
        endcase
    end

endmodule
