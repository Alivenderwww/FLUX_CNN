// =============================================================================
// tb_bd_top.sv  --  VD100 BD-level ModelSim testbench
//
// 目的: 在 sim 中复现 board layer 0 OFM=166KB (232/240 row) stuck.
//
// Sim 路径完全跟 board 一致:
//   ConvCore m_axi → smartconnect_pl_X → axi_noc.S0X → axi_noc DDRMC → DDR4 sim model
//
// 策略 (avoiding cips VIP 复杂 API):
//   sys_clk 自动 drive, BD 内部 cips_vip 自动 boot, 释放 PL reset.
//   testbench 用 SystemVerilog hier-ref + force 直接写 ConvCore 内部 cfg_regs registers,
//   相当于 board PEEK/POKE 通过 csr_axil 的 simulator 等价.
//   然后 force start_dfe / start_layer pulse 触发 layer 0.
//
// Hier path:
//   tb_bd_top.dut.design_1_i.u_mc_vd100.inst.u_inner.u_inner.gen_core[0].u_core
//                                       ^bd_v   ^bd      ^vd100  ^generate
// =============================================================================

`timescale 1ns/1ps

module tb_bd_top;

    // ================== Clock + DDR4 dangling ports ===========================
    reg sys_clk_p = 1'b0;
    reg sys_clk_n = 1'b1;
    always #2500 begin   // 200 MHz diff clk (5 ns / 2 = 2500 ps half period)
        sys_clk_p = ~sys_clk_p;
        sys_clk_n = ~sys_clk_n;
    end

    wire [0:0]  DDR4_act_n;
    wire [16:0] DDR4_adr;
    wire [1:0]  DDR4_ba;
    wire [0:0]  DDR4_bg;
    wire [0:0]  DDR4_ck_c;
    wire [0:0]  DDR4_ck_t;
    wire [0:0]  DDR4_cke;
    wire [0:0]  DDR4_cs_n;
    wire [7:0]  DDR4_dm_n;
    wire [63:0] DDR4_dq;
    wire [7:0]  DDR4_dqs_c;
    wire [7:0]  DDR4_dqs_t;
    wire [0:0]  DDR4_odt;
    wire [0:0]  DDR4_reset_n;

    design_1_wrapper dut (
        .DDR4_act_n   (DDR4_act_n),
        .DDR4_adr     (DDR4_adr),
        .DDR4_ba      (DDR4_ba),
        .DDR4_bg      (DDR4_bg),
        .DDR4_ck_c    (DDR4_ck_c),
        .DDR4_ck_t    (DDR4_ck_t),
        .DDR4_cke     (DDR4_cke),
        .DDR4_cs_n    (DDR4_cs_n),
        .DDR4_dm_n    (DDR4_dm_n),
        .DDR4_dq      (DDR4_dq),
        .DDR4_dqs_c   (DDR4_dqs_c),
        .DDR4_dqs_t   (DDR4_dqs_t),
        .DDR4_odt     (DDR4_odt),
        .DDR4_reset_n (DDR4_reset_n),
        .sys_clk_n    (sys_clk_n),
        .sys_clk_p    (sys_clk_p)
    );

    // ================== Hier-ref alias 简化 path ==============================
    // 注意: SystemVerilog `bind` / `bind_alias` 在 ModelSim 也支持, 但 hier-ref 更稳.
    `define CORE0 dut.design_1_i.u_mc_vd100.inst.u_inner.u_inner.gen_core[0].u_core

    // ================== Heartbeat + 状态 print ================================
    initial begin
        $display("[t=%0t] tb_bd_top sim start", $time);
        // 等 PL reset 释放 (cips boot ~几 us)
        #1_000_000;   // 1 us heartbeat
        $display("[t=%0t] PL reset 状态: %b", $time, `CORE0.rst_n);
        forever begin
            #500_000;   // 0.5 us mark
            $display("[t=%0t] c0 seq_state=%0d iSG_st=%0d cmd_idx=%0d r_rows_pushed=%0d rows_consumed=%0d | oSG_state=%0d ocmd_idx=%0d row_dr=%0d",
                     $time,
                     `CORE0.u_sequencer.state,
                     `CORE0.g_idma_sg.u_idma_sg.st,
                     `CORE0.g_idma_sg.u_idma_sg.r_cmd_idx,
                     `CORE0.g_idma_sg.u_idma_sg.r_rows_pushed,
                     `CORE0.rows_consumed,
                     `CORE0.g_odma_sg.u_odma_sg.state,
                     `CORE0.g_odma_sg.u_odma_sg.r_cmd_idx,
                     `CORE0.g_odma_sg.u_odma_sg.r_rows_drained);
        end
    end

    // ================== Watchdog ===============================================
    initial begin
        #(10_000_000_000.0);   // 10 ms sim time watchdog
        $display("FATAL: tb_bd_top watchdog @ %0t", $time);
        $stop;
    end

    // ================== TODO Phase: trigger ConvCore =========================
    // Phase 1: verify cips boot + PL reset deassert (heartbeat 观察 rst_n)
    // Phase 2: hier-ref write cfg_regs 数据 + DDR backdoor 写 IFM/WB/desc
    // Phase 3: hier-ref force start_dfe / start_layer pulse
    // Phase 4: 观察 232 row OFM stuck 是否复现

endmodule
