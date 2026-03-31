// ============================================================================
// Spatz Banked Scratchpad Memory (TCDM)
// Reference: hw/system/spatz_cluster/src/spatz_cluster.sv lines 594-630
//
// 16 independently-accessible SRAM banks.
// Each bank: BANK_DEPTH words ? DATA_W bits
// Total: 16 ? 512 ? 128 bits = 128 KB
//
// Single-ported per bank per cycle (1R or 1W, not both).
// Bank selection: addr[BYTE_OFF +: BANK_SEL_W] (interleaved)
// ============================================================================
module spatz_spm #(
    parameter DATA_W      = 128,
    parameter N_BANKS     = 16,
    parameter BANK_DEPTH  = 512,
    parameter ADDR_W      = 17
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Per-bank interface (from crossbar)
    input  wire [N_BANKS-1:0]                  bank_cs,     // chip select
    input  wire [N_BANKS-1:0]                  bank_we,     // write enable
    input  wire [N_BANKS*9-1:0]                bank_addr,   // 9-bit bank offset (clog2(512))
    input  wire [N_BANKS*DATA_W-1:0]           bank_wdata,
    output reg  [N_BANKS*DATA_W-1:0]           bank_rdata
);

    localparam BANK_ADDR_W = 9; // clog2(512)

    // Storage: 16 banks ? 512 words ? 128 bits
    reg [DATA_W-1:0] mem [0:N_BANKS-1][0:BANK_DEPTH-1];

    integer b;
    always @(posedge clk) begin
        for (b = 0; b < N_BANKS; b = b + 1) begin
            if (bank_cs[b]) begin
                if (bank_we[b])
                    mem[b][bank_addr[b*BANK_ADDR_W +: BANK_ADDR_W]]
                        <= bank_wdata[b*DATA_W +: DATA_W];
                else
                    bank_rdata[b*DATA_W +: DATA_W]
                        <= mem[b][bank_addr[b*BANK_ADDR_W +: BANK_ADDR_W]];
            end
        end
    end

    // Initialize to zero (for simulation)
    integer bi, bj;
    initial begin
        for (bi = 0; bi < N_BANKS; bi = bi + 1)
            for (bj = 0; bj < BANK_DEPTH; bj = bj + 1)
                mem[bi][bj] = {DATA_W{1'b0}};
        bank_rdata = {(N_BANKS*DATA_W){1'b0}};
    end

endmodule