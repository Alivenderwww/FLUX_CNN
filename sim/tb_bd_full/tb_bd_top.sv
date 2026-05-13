// =============================================================================
// tb_bd_top.sv  --  VD100 BD-level ModelSim testbench
//
// 目的: 在 sim 中复现 board layer 0 OFM=166KB (232/240 row) stuck.
//
// Sim 路径完全跟 board 一致:
//   ConvCore m_axi → smartconnect_pl_X → axi_noc.S0X → axi_noc DDRMC → DDR4 sim model
//   cips_vip (替代 a72) → smartconnect_0 → ConvCore csr_axil
//
// 策略 (avoiding cips VIP 复杂 API):
//   sys_clk 自动 drive, BD 内部 cips_vip 自动 boot, 释放 PL reset.
//   testbench 用 SystemVerilog hier-ref + force 直接写 ConvCore 内部 cfg_regs registers
//   (相当于 board PEEK/POKE 通过 csr_axil 的 simulator 等价).
//   然后 force start_dfe / start_layer pulse 触发 layer 0.
//
// 注意: DDR4 数据 (IFM/WB/RDMA/desc/SG cmd) sim 内无法用 deploy_smc_case.py 写入 DDR.
// 我们需要 通过 cips VIP backdoor 写 DDR (BD axi_noc 提供 backdoor API), 或者
// hier-ref 写 axi_noc DDR responder 内部 mem array.
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

    // DDR4 物理 ports (axi_noc 内部 DDR responder 自己驱动, 这些 wire 是顶层接出)
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

    // ================== DUT (design_1_wrapper) =================================
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

    // ================== Phase 监控 + 时序 trace ================================
    // Hier-ref ConvCore[0] sequencer / dispatcher internal state via design_1 BD
    // hierarchy: dut → design_1_i → u_mc_vd100 → inst → (multicore_top_vd100_bd)
    //   → gen_core[0].u_core → u_sequencer / g_idma_sg.u_idma_sg / etc
    //
    // 周期性 print sequencer state + dispatcher.st + cmd_idx, 跟 board PEEK 一致.
    // TODO: hier-ref print 之后再加 (需要 verify generate block instance name)
    initial begin
        $display("[t=%0t] tb_bd_top sim start", $time);
        forever begin
            #100_000;   // 100 ns mark
            $display("[t=%0t] heartbeat", $time);
        end
    end

    // ================== Watchdog ===============================================
    initial begin
        #(10_000_000_000.0);   // 10 ms sim time watchdog (real literal)
        $display("FATAL: tb_bd_top watchdog @ %0t", $time);
        $stop;
    end

    // ================== TODO: cips_vip write_data 序列 ========================
    // 后续: 用 dut.design_1_i.versal_cips_0.inst.<axi_vip_path>.write_data(addr, data)
    //  替代 a72 baremetal 的 deploy_smc_case.py + start_dfe / start_layer.
    // 或者: hier-ref force ConvCore cfg_regs 内部 reg 后释放 + assert start_dfe pulse.
    //
    // Phase 1: 让 sim 跑 1 ms 看 cips boot + clk_wizard 输出 PL_CLK 是否启动.
    //   预期: dut.design_1_i.proc_sys_reset_0.peripheral_aresetn 拉高.
    //
    // Phase 2: write csr_axil 触发 start_dfe (deploy_smc_case Phase 1)
    // Phase 3: 等 dfe_done, write start_layer (Phase 2)
    // Phase 4: 每 1us print seq/iSG/oSG state, 找 232 row stuck 点.

endmodule
