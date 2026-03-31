`timescale 1ns / 1ps
module spatz_vrf #(
    parameter ELEN      = 32,
    parameter N_FU      = 4,
    parameter VLEN      = 512,
    parameter NRVREG    = 32,
    parameter NR_RPORTS = 3,
    parameter NR_WPORTS = 3
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire [NR_WPORTS*7-1:0]           waddr_i,
    input  wire [NR_WPORTS*N_FU*ELEN-1:0]   wdata_i,
    input  wire [NR_WPORTS-1:0]             we_i,
    output reg  [NR_WPORTS-1:0]             wvalid_o,
    input  wire [NR_RPORTS*7-1:0]           raddr_i,
    input  wire [NR_RPORTS-1:0]             re_i,
    output reg  [NR_RPORTS*N_FU*ELEN-1:0]   rdata_o,
    output reg  [NR_RPORTS-1:0]             rvalid_o
);

    localparam WORD_W          = N_FU * ELEN;
    localparam WORDS_PER_VREG  = VLEN / WORD_W;
    localparam NR_VRF_WORDS    = NRVREG * WORDS_PER_VREG;
    localparam NR_BANKS        = 4;
    localparam WORDS_PER_BANK  = NR_VRF_WORDS / NR_BANKS;
    localparam BANK_SEL_W      = 2;
    localparam BANK_ADDR_W     = 5;

    reg [WORD_W-1:0] bank [0:NR_BANKS-1][0:WORDS_PER_BANK-1];

    function [BANK_SEL_W-1:0] f_bank;
        input [6:0] addr;
        f_bank = addr[BANK_SEL_W-1:0];
    endfunction

    function [BANK_ADDR_W-1:0] f_offset;
        input [6:0] addr;
        f_offset = addr[BANK_SEL_W +: BANK_ADDR_W];
    endfunction

    wire [6:0]              w_addr [0:NR_WPORTS-1];
    wire [BANK_SEL_W-1:0]  w_bank [0:NR_WPORTS-1];
    wire [BANK_ADDR_W-1:0] w_off  [0:NR_WPORTS-1];
    wire [6:0]              r_addr [0:NR_RPORTS-1];
    wire [BANK_SEL_W-1:0]  r_bank [0:NR_RPORTS-1];
    wire [BANK_ADDR_W-1:0] r_off  [0:NR_RPORTS-1];

    genvar gi;
    generate
        for (gi = 0; gi < NR_WPORTS; gi = gi + 1) begin : gen_wdec
            assign w_addr[gi] = waddr_i[gi*7 +: 7];
            assign w_bank[gi] = f_bank(w_addr[gi]);
            assign w_off[gi]  = f_offset(w_addr[gi]);
        end
        for (gi = 0; gi < NR_RPORTS; gi = gi + 1) begin : gen_rdec
            assign r_addr[gi] = raddr_i[gi*7 +: 7];
            assign r_bank[gi] = f_bank(r_addr[gi]);
            assign r_off[gi]  = f_offset(r_addr[gi]);
        end
    endgenerate

    // Write arbitration (combinational)
    reg [NR_BANKS-1:0] bank_w_taken;
    integer wp;
    always @(*) begin
        bank_w_taken = {NR_BANKS{1'b0}};
        wvalid_o     = {NR_WPORTS{1'b0}};
        for (wp = 0; wp < NR_WPORTS; wp = wp + 1) begin
            if (we_i[wp] && !bank_w_taken[w_bank[wp]]) begin
                bank_w_taken[w_bank[wp]] = 1'b1;
                wvalid_o[wp] = 1'b1;
            end
        end
    end

    // Writes: registered
    integer wi;
    always @(posedge clk) begin
        for (wi = 0; wi < NR_WPORTS; wi = wi + 1)
            if (wvalid_o[wi])
                bank[w_bank[wi]][w_off[wi]] <= wdata_i[wi*WORD_W +: WORD_W];
    end

    // Reads: COMBINATIONAL (zero-cycle latency)
    // rvalid and rdata available in the SAME cycle as re
    integer ri;
    always @(*) begin
        rvalid_o = {NR_RPORTS{1'b0}};
        rdata_o  = {(NR_RPORTS*WORD_W){1'b0}};
        for (ri = 0; ri < NR_RPORTS; ri = ri + 1) begin
            if (re_i[ri]) begin
                rdata_o[ri*WORD_W +: WORD_W] = bank[r_bank[ri]][r_off[ri]];
                rvalid_o[ri] = 1'b1;
            end
        end
    end

    integer ib, io;
    initial begin
        for (ib = 0; ib < NR_BANKS; ib = ib + 1)
            for (io = 0; io < WORDS_PER_BANK; io = io + 1)
                bank[ib][io] = {WORD_W{1'b0}};
    end

endmodule