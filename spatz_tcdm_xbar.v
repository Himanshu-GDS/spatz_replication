`timescale 1ns / 1ps
// ============================================================================
// Spatz TCDM Crossbar + Integrated SPM Banks
// ============================================================================
module spatz_tcdm_xbar #(
    parameter DATA_W     = 128,
    parameter N_PORTS    = 2,
    parameter N_BANKS    = 16,
    parameter BANK_DEPTH = 512,
    parameter ADDR_W     = 17
)(
    input  wire                            clk,
    input  wire                            rst_n,
    input  wire [N_PORTS-1:0]              req_valid,
    input  wire [N_PORTS*ADDR_W-1:0]       req_addr,
    input  wire [N_PORTS*DATA_W-1:0]       req_wdata,
    input  wire [N_PORTS-1:0]              req_we,
    output reg  [N_PORTS-1:0]              req_gnt,
    output reg  [N_PORTS*DATA_W-1:0]       req_rdata,
    output reg  [31:0]                     bank_conflict_cnt,
    output reg  [31:0]                     total_access_cnt
);

    localparam BYTE_OFF_W  = 4;
    localparam BANK_SEL_W  = 4;
    localparam BANK_ADDR_W = 9;

    reg [DATA_W-1:0] bank_mem [0:N_BANKS-1][0:BANK_DEPTH-1];

    wire [ADDR_W-1:0]      p_addr   [0:N_PORTS-1];
    wire [BANK_SEL_W-1:0]  p_bank   [0:N_PORTS-1];
    wire [BANK_ADDR_W-1:0] p_offset [0:N_PORTS-1];

    genvar gp;
    generate
        for (gp = 0; gp < N_PORTS; gp = gp + 1) begin : gen_addr_decode
            assign p_addr[gp]   = req_addr[gp*ADDR_W +: ADDR_W];
            assign p_bank[gp]   = p_addr[gp][BYTE_OFF_W +: BANK_SEL_W];
            assign p_offset[gp] = p_addr[gp][BYTE_OFF_W+BANK_SEL_W +: BANK_ADDR_W];
        end
    endgenerate

    reg [N_BANKS-1:0] bank_taken;
    integer arb_p;
    always @(*) begin
        bank_taken = {N_BANKS{1'b0}};
        req_gnt    = {N_PORTS{1'b0}};
        for (arb_p = 0; arb_p < N_PORTS; arb_p = arb_p + 1) begin
            if (req_valid[arb_p] && !bank_taken[p_bank[arb_p]]) begin
                bank_taken[p_bank[arb_p]] = 1'b1;
                req_gnt[arb_p] = 1'b1;
            end
        end
    end

    integer mem_p;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bank_conflict_cnt <= 32'd0;
            total_access_cnt  <= 32'd0;
            req_rdata         <= {(N_PORTS*DATA_W){1'b0}};
        end else begin
            for (mem_p = 0; mem_p < N_PORTS; mem_p = mem_p + 1) begin
                if (req_valid[mem_p])
                    total_access_cnt <= total_access_cnt + 1;
                if (req_valid[mem_p] && !req_gnt[mem_p])
                    bank_conflict_cnt <= bank_conflict_cnt + 1;
                if (req_gnt[mem_p]) begin
                    if (req_we[mem_p])
                        bank_mem[p_bank[mem_p]][p_offset[mem_p]]
                            <= req_wdata[mem_p*DATA_W +: DATA_W];
                    else
                        req_rdata[mem_p*DATA_W +: DATA_W]
                            <= bank_mem[p_bank[mem_p]][p_offset[mem_p]];
                end
            end
        end
    end

    integer bi, bj;
    initial begin
        for (bi = 0; bi < N_BANKS; bi = bi + 1)
            for (bj = 0; bj < BANK_DEPTH; bj = bj + 1)
                bank_mem[bi][bj] = {DATA_W{1'b0}};
    end

endmodule