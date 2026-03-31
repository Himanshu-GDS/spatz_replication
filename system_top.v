`timescale 1ns / 1ps
module system_top #(
    parameter ELEN       = 32,
    parameter N_FU       = 4,
    parameter VLEN       = 512,
    parameter N_BANKS    = 16,
    parameter BANK_DEPTH = 512,
    parameter SPM_ADDR_W = 17,
    parameter N_PORTS    = 3
)(
    input  wire clk,
    input  wire rst,         // Active high: resets everything
    input  wire cpu_enable,  // When low, RISC-V core is held in reset

    // DMA interface
    input  wire        dma_start,
    output wire        dma_done,
    input  wire [SPM_ADDR_W-1:0] dma_dst_addr,
    input  wire [15:0] dma_length,
    input  wire [SPM_ADDR_W-1:0] dma_stride,
    input  wire [15:0] dma_src_offset,
    output wire [15:0] dma_ext_idx,
    input  wire [31:0] dma_ext_data,

    // Status
    output wire        done,
    output wire        perf_enable
);

    wire rst_n = ~rst;

    // RISC-V reset: held in reset until cpu_enable goes high
    wire cpu_rst = rst | (~cpu_enable);

    // RISC-V <-> Bridge wires
    wire        vec_cmd_valid;
    wire [31:0] vec_instr;
    wire [31:0] vec_rs1_data;
    wire        vec_stall;

    // Bridge <-> Spatz wires
    wire        spatz_cmd_valid;
    wire        spatz_cmd_ready;
    wire [2:0]  spatz_cmd_op;
    wire [4:0]  spatz_cmd_vs1, spatz_cmd_vs2, spatz_cmd_vd;
    wire [SPM_ADDR_W-1:0] spatz_cmd_spm_addr;
    wire        spatz_busy;

    wire cc0_vau_busy, cc1_vau_busy;
    wire cc0_vlsu_busy, cc1_vlsu_busy;
    wire cc0_vsldu_busy, cc1_vsldu_busy;
    wire cc0_stall, cc1_stall;

    // Performance counter enable
    reg perf_en_r;
    always @(posedge clk) begin
        if (rst) perf_en_r <= 0;
        else if (vec_cmd_valid) perf_en_r <= 1;
        else if (done) perf_en_r <= 0;
    end
    assign perf_enable = perf_en_r;

    // ================================================================
    // RISC-V Core (uses cpu_rst, not rst)
    // ================================================================
    riscv_top u_riscv (
        .rst(cpu_rst),
        .clk(clk),
        .vec_stall(vec_stall),
        .vec_cmd_valid(vec_cmd_valid),
        .vec_instr(vec_instr),
        .vec_rs1_data(vec_rs1_data),
        .done(done)
    );

    // ================================================================
    // Vector Command Bridge
    // ================================================================
    vector_cmd_bridge u_bridge (
        .clk(clk),
        .rst_n(rst_n),
        .vec_cmd_valid(vec_cmd_valid),
        .vec_instr(vec_instr),
        .vec_rs1_data(vec_rs1_data),
        .spatz_cmd_valid(spatz_cmd_valid),
        .spatz_cmd_ready(spatz_cmd_ready),
        .spatz_cmd_op(spatz_cmd_op),
        .spatz_cmd_vs1(spatz_cmd_vs1),
        .spatz_cmd_vs2(spatz_cmd_vs2),
        .spatz_cmd_vd(spatz_cmd_vd),
        .spatz_cmd_spm_addr(spatz_cmd_spm_addr),
        .vec_stall(vec_stall),
        .spatz_busy(spatz_busy)
    );

    // ================================================================
    // Spatz Cluster (uses rst_n, released when rst goes low)
    // ================================================================
    spatz_cluster_top #(
        .ELEN(ELEN), .N_FU(N_FU), .VLEN(VLEN),
        .N_BANKS(N_BANKS), .BANK_DEPTH(BANK_DEPTH),
        .SPM_ADDR_W(SPM_ADDR_W), .N_PORTS(N_PORTS)
    ) u_spatz (
        .clk(clk),
        .rst_n(rst_n),
        .cc0_cmd_valid(spatz_cmd_valid),
        .cc0_cmd_ready(spatz_cmd_ready),
        .cc0_cmd_op(spatz_cmd_op),
        .cc0_cmd_vs1(spatz_cmd_vs1),
        .cc0_cmd_vs2(spatz_cmd_vs2),
        .cc0_cmd_vd(spatz_cmd_vd),
        .cc0_cmd_spm_addr(spatz_cmd_spm_addr),
        .cc0_busy(spatz_busy),
        .cc1_cmd_valid(1'b0),
        .cc1_cmd_ready(),
        .cc1_cmd_op(3'b0),
        .cc1_cmd_vs1(5'b0),
        .cc1_cmd_vs2(5'b0),
        .cc1_cmd_vd(5'b0),
        .cc1_cmd_spm_addr({SPM_ADDR_W{1'b0}}),
        .cc1_busy(),
        .dma_start(dma_start),
        .dma_done(dma_done),
        .dma_dst_addr(dma_dst_addr),
        .dma_length(dma_length),
        .dma_stride(dma_stride),
        .dma_src_offset(dma_src_offset),
        .dma_ext_idx(dma_ext_idx),
        .dma_ext_data(dma_ext_data),
        .perf_enable(perf_en_r),
        .cc0_vau_busy(cc0_vau_busy),
        .cc1_vau_busy(cc1_vau_busy),
        .cc0_vlsu_busy(cc0_vlsu_busy),
        .cc1_vlsu_busy(cc1_vlsu_busy),
        .cc0_vsldu_busy(cc0_vsldu_busy),
        .cc1_vsldu_busy(cc1_vsldu_busy),
        .cc0_stall(cc0_stall),
        .cc1_stall(cc1_stall)
    );

endmodule