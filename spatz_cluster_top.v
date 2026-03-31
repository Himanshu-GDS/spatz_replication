`timescale 1ns / 1ps
// ============================================================================
// Spatz Cluster Top -- 2 CCs + Crossbar + DMA + Perf Counters
// ============================================================================
module spatz_cluster_top #(
    parameter ELEN       = 32,
    parameter N_FU       = 4,
    parameter VLEN       = 512,
    parameter N_BANKS    = 16,
    parameter BANK_DEPTH = 512,
    parameter SPM_ADDR_W = 17,
    parameter N_PORTS    = 3
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cc0_cmd_valid, output wire cc0_cmd_ready,
    input  wire [2:0]  cc0_cmd_op,
    input  wire [4:0]  cc0_cmd_vs1, cc0_cmd_vs2, cc0_cmd_vd,
    input  wire [SPM_ADDR_W-1:0] cc0_cmd_spm_addr,
    output wire        cc0_busy,

    input  wire        cc1_cmd_valid, output wire cc1_cmd_ready,
    input  wire [2:0]  cc1_cmd_op,
    input  wire [4:0]  cc1_cmd_vs1, cc1_cmd_vs2, cc1_cmd_vd,
    input  wire [SPM_ADDR_W-1:0] cc1_cmd_spm_addr,
    output wire        cc1_busy,

    input  wire                    dma_start,
    output wire                    dma_done,
    input  wire [SPM_ADDR_W-1:0]  dma_dst_addr,
    input  wire [15:0]             dma_length,
    input  wire [SPM_ADDR_W-1:0]  dma_stride,
    input  wire [15:0]             dma_src_offset,
    output wire [15:0]             dma_ext_idx,
    input  wire [31:0]             dma_ext_data,

    input  wire                    perf_enable,

    output wire        cc0_vau_busy, cc1_vau_busy,
    output wire        cc0_vlsu_busy, cc1_vlsu_busy,
    output wire        cc0_vsldu_busy, cc1_vsldu_busy,
    output wire        cc0_stall, cc1_stall
);

    localparam WORD_W = N_FU * ELEN;

    wire [SPM_ADDR_W-1:0] cc0_spm_addr, cc1_spm_addr;
    wire [WORD_W-1:0]     cc0_spm_wdata, cc1_spm_wdata;
    wire                   cc0_spm_we, cc1_spm_we;
    wire                   cc0_spm_req, cc1_spm_req;
    wire [WORD_W-1:0]     cc0_spm_rdata, cc1_spm_rdata;
    wire                   cc0_spm_gnt, cc1_spm_gnt;

    wire [SPM_ADDR_W-1:0] dma_tcdm_addr;
    wire [WORD_W-1:0]     dma_tcdm_wdata;
    wire                   dma_tcdm_we, dma_tcdm_req, dma_tcdm_gnt;

    spatz_core_complex #(.ELEN(ELEN),.N_FU(N_FU),.VLEN(VLEN),.SPM_ADDR_W(SPM_ADDR_W),.CC_ID(0))
    u_cc0 (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cc0_cmd_valid), .cmd_ready(cc0_cmd_ready),
        .cmd_op(cc0_cmd_op), .cmd_vs1(cc0_cmd_vs1), .cmd_vs2(cc0_cmd_vs2), .cmd_vd(cc0_cmd_vd),
        .cmd_spm_addr(cc0_cmd_spm_addr),
        .spm_addr(cc0_spm_addr), .spm_wdata(cc0_spm_wdata),
        .spm_we(cc0_spm_we), .spm_req(cc0_spm_req),
        .spm_rdata(cc0_spm_rdata), .spm_gnt(cc0_spm_gnt),
        .busy(cc0_busy), .vau_busy(cc0_vau_busy),
        .vlsu_busy(cc0_vlsu_busy), .vsldu_busy(cc0_vsldu_busy), .ctrl_stall(cc0_stall)
    );

    spatz_core_complex #(.ELEN(ELEN),.N_FU(N_FU),.VLEN(VLEN),.SPM_ADDR_W(SPM_ADDR_W),.CC_ID(1))
    u_cc1 (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cc1_cmd_valid), .cmd_ready(cc1_cmd_ready),
        .cmd_op(cc1_cmd_op), .cmd_vs1(cc1_cmd_vs1), .cmd_vs2(cc1_cmd_vs2), .cmd_vd(cc1_cmd_vd),
        .cmd_spm_addr(cc1_cmd_spm_addr),
        .spm_addr(cc1_spm_addr), .spm_wdata(cc1_spm_wdata),
        .spm_we(cc1_spm_we), .spm_req(cc1_spm_req),
        .spm_rdata(cc1_spm_rdata), .spm_gnt(cc1_spm_gnt),
        .busy(cc1_busy), .vau_busy(cc1_vau_busy),
        .vlsu_busy(cc1_vlsu_busy), .vsldu_busy(cc1_vsldu_busy), .ctrl_stall(cc1_stall)
    );

    spatz_dma #(.DATA_W(WORD_W), .ADDR_W(SPM_ADDR_W))
    u_dma (
        .clk(clk), .rst_n(rst_n),
        .dma_start(dma_start), .dma_done(dma_done),
        .dma_dst_addr(dma_dst_addr), .dma_length(dma_length),
        .dma_stride(dma_stride), .dma_src_offset(dma_src_offset),
        .dma_ext_idx(dma_ext_idx), .dma_ext_data(dma_ext_data),
        .tcdm_addr(dma_tcdm_addr), .tcdm_wdata(dma_tcdm_wdata),
        .tcdm_we(dma_tcdm_we), .tcdm_req(dma_tcdm_req),
        .tcdm_gnt(dma_tcdm_gnt)
    );

    wire [N_PORTS-1:0]             xbar_valid = {dma_tcdm_req, cc1_spm_req, cc0_spm_req};
    wire [N_PORTS*SPM_ADDR_W-1:0] xbar_addr  = {dma_tcdm_addr, cc1_spm_addr, cc0_spm_addr};
    wire [N_PORTS*WORD_W-1:0]     xbar_wdata = {dma_tcdm_wdata, cc1_spm_wdata, cc0_spm_wdata};
    wire [N_PORTS-1:0]            xbar_we    = {dma_tcdm_we, cc1_spm_we, cc0_spm_we};
    wire [N_PORTS-1:0]            xbar_gnt;
    wire [N_PORTS*WORD_W-1:0]    xbar_rdata;
    wire [31:0]                   conflict_cnt, access_cnt;

    spatz_tcdm_xbar #(.DATA_W(WORD_W),.N_PORTS(N_PORTS),.N_BANKS(N_BANKS),
                       .BANK_DEPTH(BANK_DEPTH),.ADDR_W(SPM_ADDR_W))
    u_xbar (
        .clk(clk), .rst_n(rst_n),
        .req_valid(xbar_valid), .req_addr(xbar_addr),
        .req_wdata(xbar_wdata), .req_we(xbar_we),
        .req_gnt(xbar_gnt), .req_rdata(xbar_rdata),
        .bank_conflict_cnt(conflict_cnt), .total_access_cnt(access_cnt)
    );

    assign cc0_spm_gnt   = xbar_gnt[0];
    assign cc1_spm_gnt   = xbar_gnt[1];
    assign dma_tcdm_gnt  = xbar_gnt[2];
    assign cc0_spm_rdata = xbar_rdata[0*WORD_W +: WORD_W];
    assign cc1_spm_rdata = xbar_rdata[1*WORD_W +: WORD_W];

    wire [2*32-1:0] pc_vau_active;
    wire [2*32-1:0] pc_vlsu_active;
    wire [2*32-1:0] pc_vsldu_active;
    wire [2*32-1:0] pc_stall_cnt;

    spatz_perf_counters #(.N_CORES(2), .N_FU(N_FU))
    u_perf (
        .clk(clk), .rst_n(rst_n),
        .enable(perf_enable),
        .tcdm_access_cnt(access_cnt), .tcdm_conflict_cnt(conflict_cnt),
        .vau_busy({cc1_vau_busy, cc0_vau_busy}),
        .vlsu_busy({cc1_vlsu_busy, cc0_vlsu_busy}),
        .vsldu_busy({cc1_vsldu_busy, cc0_vsldu_busy}),
        .stall({cc1_stall, cc0_stall}),
        .cnt_cycles(),
        .cnt_vau_active(pc_vau_active),
        .cnt_vlsu_active(pc_vlsu_active),
        .cnt_vsldu_active(pc_vsldu_active),
        .cnt_stall(pc_stall_cnt),
        .cnt_flops()
    );

endmodule