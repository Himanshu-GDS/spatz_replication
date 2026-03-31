`timescale 1ns / 1ps
// ============================================================================
// Spatz Vector Slide Unit (VSLDU)
// SLIDE_UP, SLIDE_DOWN, VMV
// ============================================================================
module spatz_vsldu #(
    parameter ELEN = 32,
    parameter N_FU = 4,
    parameter VLEN = 512
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    cmd_valid,
    output wire                    cmd_ready,
    input  wire [1:0]              cmd_op,
    input  wire [4:0]              cmd_vs2,
    input  wire [4:0]              cmd_vd,
    output reg  [6:0]              vrf_raddr,
    output reg                     vrf_re,
    input  wire [N_FU*ELEN-1:0]   vrf_rdata,
    input  wire                    vrf_rvalid,
    output reg  [6:0]              vrf_waddr,
    output reg  [N_FU*ELEN-1:0]   vrf_wdata,
    output reg                     vrf_we,
    input  wire                    vrf_wvalid,
    output wire                    busy
);

    localparam WORD_W         = N_FU * ELEN;
    localparam WORDS_PER_VREG = VLEN / WORD_W;
    localparam TOTAL_ELEMS    = VLEN / ELEN;

    localparam S_IDLE  = 3'd0;
    localparam S_READ  = 3'd1;
    localparam S_READW = 3'd2;
    localparam S_CALC  = 3'd3;
    localparam S_WRITE = 3'd4;
    localparam S_NEXT  = 3'd5;
    localparam S_DONE  = 3'd6;

    reg [2:0] state;
    reg [1:0] op_r;
    reg [4:0] vs2_r, vd_r;
    reg [2:0] rd_cnt, wr_cnt;

    reg [ELEN-1:0] src_vec [0:TOTAL_ELEMS-1];
    reg [ELEN-1:0] dst_vec [0:TOTAL_ELEMS-1];

    assign busy      = (state != S_IDLE);
    assign cmd_ready = (state == S_IDLE);

    integer iw, ie;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            rd_cnt <= 0; wr_cnt <= 0;
            vrf_re <= 0; vrf_we <= 0;
        end else begin
            vrf_re <= 0; vrf_we <= 0;
            case (state)
                S_IDLE: if (cmd_valid) begin
                    op_r <= cmd_op; vs2_r <= cmd_vs2; vd_r <= cmd_vd;
                    rd_cnt <= 0; wr_cnt <= 0;
                    state <= S_READ;
                end

                S_READ: begin
                    vrf_raddr <= {vs2_r, rd_cnt[1:0]};
                    vrf_re <= 1;
                    state <= S_READW;
                end

                S_READW: begin
                    if (vrf_rvalid) begin
                        for (iw = 0; iw < N_FU; iw = iw + 1)
                            src_vec[rd_cnt*N_FU + iw] <= vrf_rdata[iw*ELEN +: ELEN];
                        if (rd_cnt == WORDS_PER_VREG - 1)
                            state <= S_CALC;
                        else begin
                            rd_cnt <= rd_cnt + 1;
                            state <= S_READ;
                        end
                    end
                end

                S_CALC: begin
                    for (ie = 0; ie < TOTAL_ELEMS; ie = ie + 1) begin
                        case (op_r)
                            2'b00: dst_vec[ie] <= (ie == 0) ? {ELEN{1'b0}} : src_vec[ie-1];
                            2'b01: dst_vec[ie] <= (ie == TOTAL_ELEMS-1) ? {ELEN{1'b0}} : src_vec[ie+1];
                            default: dst_vec[ie] <= src_vec[ie];
                        endcase
                    end
                    wr_cnt <= 0;
                    state <= S_WRITE;
                end

                S_WRITE: begin
                    vrf_waddr <= {vd_r, wr_cnt[1:0]};
                    for (iw = 0; iw < N_FU; iw = iw + 1)
                        vrf_wdata[iw*ELEN +: ELEN] <= dst_vec[wr_cnt*N_FU + iw];
                    vrf_we <= 1;
                    state <= S_NEXT;
                end

                S_NEXT: begin
                    if (wr_cnt == WORDS_PER_VREG - 1)
                        state <= S_DONE;
                    else begin
                        wr_cnt <= wr_cnt + 1;
                        state <= S_WRITE;
                    end
                end

                S_DONE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule