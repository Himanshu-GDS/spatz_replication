`timescale 1ns / 1ps
// ============================================================================
// Spatz Performance Counter Block
// ============================================================================
module spatz_perf_counters #(
    parameter N_CORES = 2,
    parameter N_FU    = 4
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        enable,
    input  wire [31:0]                 tcdm_access_cnt,
    input  wire [31:0]                 tcdm_conflict_cnt,
    input  wire [N_CORES-1:0]          vau_busy,
    input  wire [N_CORES-1:0]          vlsu_busy,
    input  wire [N_CORES-1:0]          vsldu_busy,
    input  wire [N_CORES-1:0]          stall,
    output reg  [31:0]                 cnt_cycles,
    output reg  [N_CORES*32-1:0]       cnt_vau_active,
    output reg  [N_CORES*32-1:0]       cnt_vlsu_active,
    output reg  [N_CORES*32-1:0]       cnt_vsldu_active,
    output reg  [N_CORES*32-1:0]       cnt_stall,
    output reg  [31:0]                 cnt_flops
);

    integer c;
    reg [31:0] flop_inc;

    always @(*) begin
        flop_inc = 32'd0;
        for (c = 0; c < N_CORES; c = c + 1)
            if (vau_busy[c]) flop_inc = flop_inc + N_FU;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_cycles       <= 0;
            cnt_flops        <= 0;
            cnt_vau_active   <= 0;
            cnt_vlsu_active  <= 0;
            cnt_vsldu_active <= 0;
            cnt_stall        <= 0;
        end else if (enable) begin
            cnt_cycles <= cnt_cycles + 1;
            cnt_flops  <= cnt_flops + flop_inc;
            for (c = 0; c < N_CORES; c = c + 1) begin
                if (vau_busy[c])   cnt_vau_active[c*32 +: 32]   <= cnt_vau_active[c*32 +: 32] + 1;
                if (vlsu_busy[c])  cnt_vlsu_active[c*32 +: 32]  <= cnt_vlsu_active[c*32 +: 32] + 1;
                if (vsldu_busy[c]) cnt_vsldu_active[c*32 +: 32] <= cnt_vsldu_active[c*32 +: 32] + 1;
                if (stall[c])      cnt_stall[c*32 +: 32]        <= cnt_stall[c*32 +: 32] + 1;
            end
        end
    end

endmodule