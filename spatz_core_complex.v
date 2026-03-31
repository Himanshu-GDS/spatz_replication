`timescale 1ns / 1ps
// ============================================================================
// Spatz Core Complex - Updated for 3-port VAU reads
// When VAU is active, it uses all 3 VRF read ports
// When VAU is idle, VLSU uses port 1, VSLDU uses port 2
// ============================================================================
module spatz_core_complex #(
    parameter ELEN       = 32,
    parameter N_FU       = 4,
    parameter VLEN       = 512,
    parameter SPM_ADDR_W = 17,
    parameter CC_ID      = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire [2:0]  cmd_op,
    input  wire [4:0]  cmd_vs1,
    input  wire [4:0]  cmd_vs2,
    input  wire [4:0]  cmd_vd,
    input  wire [SPM_ADDR_W-1:0] cmd_spm_addr,
    output wire [SPM_ADDR_W-1:0]  spm_addr,
    output wire [N_FU*ELEN-1:0]   spm_wdata,
    output wire                    spm_we,
    output wire                    spm_req,
    input  wire [N_FU*ELEN-1:0]   spm_rdata,
    input  wire                    spm_gnt,
    output wire        busy,
    output wire        vau_busy,
    output wire        vlsu_busy,
    output wire        vsldu_busy,
    output wire        ctrl_stall
);

    localparam WORD_W = N_FU * ELEN;

    // ---- VRF bus wires ----
    wire [3*7-1:0]       vrf_raddr;
    wire [2:0]           vrf_re;
    wire [3*WORD_W-1:0]  vrf_rdata;
    wire [2:0]           vrf_rvalid;
    wire [3*7-1:0]       vrf_waddr;
    wire [3*WORD_W-1:0]  vrf_wdata;
    wire [2:0]           vrf_we;
    wire [2:0]           vrf_wvalid;

    // ---- Controller wires ----
    wire        vau_cv, vau_cr;
    wire [1:0]  vau_cop;
    wire [4:0]  vau_cvs1, vau_cvs2, vau_cvd;

    wire        vlsu_cv, vlsu_cr, vlsu_cld, vlsu_done;
    wire [4:0]  vlsu_cvr;
    wire [SPM_ADDR_W-1:0] vlsu_ca;

    wire        vsldu_cv, vsldu_cr;
    wire [1:0]  vsldu_cop;
    wire [4:0]  vsldu_cvs2, vsldu_cvd;

    // ---- VAU port wires (3 read ports + 1 write port) ----
    wire [6:0]         vau_raddr_0, vau_raddr_1, vau_raddr_2;
    wire               vau_re_0, vau_re_1, vau_re_2;
    wire [6:0]         vau_waddr;
    wire [WORD_W-1:0]  vau_wdata;
    wire               vau_we;

    // ---- VLSU port wires (1 read port + 1 write port) ----
    wire [6:0]         vlsu_raddr;
    wire               vlsu_re;
    wire [6:0]         vlsu_waddr;
    wire [WORD_W-1:0]  vlsu_wdata;
    wire               vlsu_we;

    // ---- VSLDU port wires (1 read port + 1 write port) ----
    wire [6:0]         vsldu_raddr;
    wire               vsldu_re;
    wire [6:0]         vsldu_waddr;
    wire [WORD_W-1:0]  vsldu_wdata;
    wire               vsldu_we;

    // ================================================================
    // VRF Read Port Muxing
    // ================================================================
    // Port 0: VAU vs1 (always owned by VAU; VLSU/VSLDU don't use it)
    // Port 1: VAU vs2 when VAU active, else VLSU
    // Port 2: VAU vd  when VAU active, else VSLDU
    //
    // Since the controller is serial (only one EU active at a time),
    // VAU active means VLSU and VSLDU are idle, so no conflict.
    // ================================================================

    // Port 0: always VAU
    wire [6:0]  mux_raddr_0 = vau_raddr_0;
    wire        mux_re_0    = vau_re_0;

    // Port 1: VAU vs2 when VAU is reading, else VLSU
    wire [6:0]  mux_raddr_1 = vau_re_1 ? vau_raddr_1 : vlsu_raddr;
    wire        mux_re_1    = vau_re_1 | vlsu_re;

    // Port 2: VAU vd when VAU is reading, else VSLDU
    wire [6:0]  mux_raddr_2 = vau_re_2 ? vau_raddr_2 : vsldu_raddr;
    wire        mux_re_2    = vau_re_2 | vsldu_re;

    assign vrf_raddr = {mux_raddr_2, mux_raddr_1, mux_raddr_0};
    assign vrf_re    = {mux_re_2, mux_re_1, mux_re_0};

    // ================================================================
    // VRF Write Port Assignment (unchanged - each EU has its own port)
    // ================================================================
    // Port 0: VAU
    // Port 1: VLSU
    // Port 2: VSLDU
    assign vrf_waddr = {vsldu_waddr, vlsu_waddr, vau_waddr};
    assign vrf_wdata = {vsldu_wdata, vlsu_wdata, vau_wdata};
    assign vrf_we    = {vsldu_we, vlsu_we, vau_we};

    // ================================================================
    // VRF Read Data Routing
    // ================================================================
    // VAU gets data from all 3 ports
    // VLSU gets data from port 1
    // VSLDU gets data from port 2
    //
    // rvalid routing:
    // VAU sees rvalid from ports 0,1,2 directly
    // VLSU sees rvalid from port 1 only when VAU is NOT using it
    // VSLDU sees rvalid from port 2 only when VAU is NOT using it

    wire vlsu_rvalid_muxed  = vrf_rvalid[1] & ~vau_busy;
    wire vsldu_rvalid_muxed = vrf_rvalid[2] & ~vau_busy;

    // ---- VRF ----
    spatz_vrf #(.ELEN(ELEN),.N_FU(N_FU),.VLEN(VLEN))
    u_vrf (
        .clk(clk), .rst_n(rst_n),
        .waddr_i(vrf_waddr), .wdata_i(vrf_wdata), .we_i(vrf_we), .wvalid_o(vrf_wvalid),
        .raddr_i(vrf_raddr), .re_i(vrf_re), .rdata_o(vrf_rdata), .rvalid_o(vrf_rvalid)
    );

    // ---- Controller ----
    spatz_controller #(.SPM_ADDR_W(SPM_ADDR_W))
    u_ctrl (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_op(cmd_op), .cmd_vs1(cmd_vs1), .cmd_vs2(cmd_vs2), .cmd_vd(cmd_vd),
        .cmd_spm_addr(cmd_spm_addr),
        .vau_cmd_valid(vau_cv), .vau_cmd_ready(vau_cr),
        .vau_cmd_op(vau_cop), .vau_cmd_vs1(vau_cvs1), .vau_cmd_vs2(vau_cvs2), .vau_cmd_vd(vau_cvd),
        .vau_busy(vau_busy),
        .vlsu_cmd_valid(vlsu_cv), .vlsu_cmd_ready(vlsu_cr),
        .vlsu_cmd_is_load(vlsu_cld), .vlsu_cmd_vreg(vlsu_cvr),
        .vlsu_cmd_spm_addr(vlsu_ca), .vlsu_busy(vlsu_busy), .vlsu_done(vlsu_done),
        .vsldu_cmd_valid(vsldu_cv), .vsldu_cmd_ready(vsldu_cr),
        .vsldu_cmd_op(vsldu_cop), .vsldu_cmd_vs2(vsldu_cvs2), .vsldu_cmd_vd(vsldu_cvd),
        .vsldu_busy(vsldu_busy),
        .busy(busy), .ctrl_stall(ctrl_stall)
    );

    // ---- VAU (3 read ports) ----
    spatz_vau #(.ELEN(ELEN),.N_FU(N_FU),.VLEN(VLEN))
    u_vau (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(vau_cv), .cmd_ready(vau_cr),
        .cmd_op(vau_cop), .cmd_vs1(vau_cvs1), .cmd_vs2(vau_cvs2), .cmd_vd(vau_cvd),
        // Read port 0: vs1
        .vrf_raddr_0(vau_raddr_0), .vrf_re_0(vau_re_0),
        .vrf_rdata_0(vrf_rdata[0*WORD_W +: WORD_W]), .vrf_rvalid_0(vrf_rvalid[0]),
        // Read port 1: vs2
        .vrf_raddr_1(vau_raddr_1), .vrf_re_1(vau_re_1),
        .vrf_rdata_1(vrf_rdata[1*WORD_W +: WORD_W]), .vrf_rvalid_1(vrf_rvalid[1]),
        // Read port 2: vd
        .vrf_raddr_2(vau_raddr_2), .vrf_re_2(vau_re_2),
        .vrf_rdata_2(vrf_rdata[2*WORD_W +: WORD_W]), .vrf_rvalid_2(vrf_rvalid[2]),
        // Write port 0
        .vrf_waddr(vau_waddr), .vrf_wdata(vau_wdata),
        .vrf_we(vau_we), .vrf_wvalid(vrf_wvalid[0]),
        .busy(vau_busy)
    );

    // ---- VLSU (uses read port 1, write port 1) ----
    spatz_vlsu #(.ELEN(ELEN),.N_FU(N_FU),.VLEN(VLEN),.SPM_ADDR_W(SPM_ADDR_W))
    u_vlsu (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(vlsu_cv), .cmd_ready(vlsu_cr),
        .cmd_is_load(vlsu_cld), .cmd_vreg(vlsu_cvr), .cmd_spm_addr(vlsu_ca),
        .vrf_raddr(vlsu_raddr), .vrf_re(vlsu_re),
        .vrf_rdata(vrf_rdata[1*WORD_W +: WORD_W]), .vrf_rvalid(vlsu_rvalid_muxed),
        .vrf_waddr(vlsu_waddr), .vrf_wdata(vlsu_wdata),
        .vrf_we(vlsu_we), .vrf_wvalid(vrf_wvalid[1]),
        .spm_addr(spm_addr), .spm_wdata(spm_wdata),
        .spm_we(spm_we), .spm_req(spm_req),
        .spm_rdata(spm_rdata), .spm_gnt(spm_gnt),
        .busy(vlsu_busy), .done(vlsu_done)
    );

    // ---- VSLDU (uses read port 2, write port 2) ----
    spatz_vsldu #(.ELEN(ELEN),.N_FU(N_FU),.VLEN(VLEN))
    u_vsldu (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(vsldu_cv), .cmd_ready(vsldu_cr),
        .cmd_op(vsldu_cop), .cmd_vs2(vsldu_cvs2), .cmd_vd(vsldu_cvd),
        .vrf_raddr(vsldu_raddr), .vrf_re(vsldu_re),
        .vrf_rdata(vrf_rdata[2*WORD_W +: WORD_W]), .vrf_rvalid(vsldu_rvalid_muxed),
        .vrf_waddr(vsldu_waddr), .vrf_wdata(vsldu_wdata),
        .vrf_we(vsldu_we), .vrf_wvalid(vrf_wvalid[2]),
        .busy(vsldu_busy)
    );

endmodule