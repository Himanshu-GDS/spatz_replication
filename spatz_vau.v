`timescale 1ns / 1ps
module spatz_vau #(
    parameter ELEN  = 32,
    parameter N_FU  = 4,
    parameter VLEN  = 512
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    cmd_valid,
    output wire                    cmd_ready,
    input  wire [1:0]              cmd_op,
    input  wire [4:0]              cmd_vs1,
    input  wire [4:0]              cmd_vs2,
    input  wire [4:0]              cmd_vd,
    output reg  [6:0]              vrf_raddr_0,
    output reg                     vrf_re_0,
    input  wire [N_FU*ELEN-1:0]   vrf_rdata_0,
    input  wire                    vrf_rvalid_0,
    output reg  [6:0]              vrf_raddr_1,
    output reg                     vrf_re_1,
    input  wire [N_FU*ELEN-1:0]   vrf_rdata_1,
    input  wire                    vrf_rvalid_1,
    output reg  [6:0]              vrf_raddr_2,
    output reg                     vrf_re_2,
    input  wire [N_FU*ELEN-1:0]   vrf_rdata_2,
    input  wire                    vrf_rvalid_2,
    output reg  [6:0]              vrf_waddr,
    output reg  [N_FU*ELEN-1:0]   vrf_wdata,
    output reg                     vrf_we,
    input  wire                    vrf_wvalid,
    output wire                    busy,
    output reg                     done
);

    localparam WORD_W         = N_FU * ELEN;
    localparam WORDS_PER_VREG = VLEN / WORD_W;

    localparam S_IDLE = 3'd0;
    localparam S_READ = 3'd1;  // Assert reads, capture data
    localparam S_EXEC = 3'd2;  // Compute from captured data
    localparam S_WB   = 3'd3;  // Write result
    localparam S_NEXT = 3'd4;  // Advance word counter

    reg [2:0] state;
    reg [1:0] op_r;
    reg [4:0] vs1_r, vs2_r, vd_r;
    reg [2:0] word_cnt;
    reg        needs_vd;

    // Captured read data (registered)
    reg [WORD_W-1:0] op_a, op_b, op_c;
    reg [WORD_W-1:0] result;

    integer idx;
    reg signed [ELEN-1:0] a_el, b_el, c_el;

    assign busy      = (state != S_IDLE);
    assign cmd_ready = (state == S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            word_cnt <= 0;
            vrf_re_0 <= 0; vrf_re_1 <= 0; vrf_re_2 <= 0;
            vrf_we   <= 0; done     <= 0;
        end else begin
            done <= 0;

            case (state)
                S_IDLE: begin
                    vrf_re_0 <= 0; vrf_re_1 <= 0; vrf_re_2 <= 0; vrf_we <= 0;
                    if (cmd_valid) begin
                        op_r     <= cmd_op;
                        vs1_r    <= cmd_vs1;
                        vs2_r    <= cmd_vs2;
                        vd_r     <= cmd_vd;
                        word_cnt <= 0;
                        needs_vd <= (cmd_op == 2'b10);
                        state    <= S_READ;
                    end
                end

                // Assert all read enables AND capture data in same cycle
                // Combinational VRF: rdata is valid while re is asserted
                S_READ: begin
                    vrf_raddr_0 <= {vs1_r, word_cnt[1:0]};
                    vrf_re_0    <= 1;
                    vrf_raddr_1 <= {vs2_r, word_cnt[1:0]};
                    vrf_re_1    <= 1;
                    if (needs_vd) begin
                        vrf_raddr_2 <= {vd_r, word_cnt[1:0]};
                        vrf_re_2    <= 1;
                    end else begin
                        vrf_re_2 <= 0;
                    end
                    vrf_we <= 0;
                    state  <= S_EXEC;
                end

                // Capture and compute
                // vrf_re was set via non-blocking in S_READ, so the VRF
                // sees re=1 THIS cycle (non-blocking updates before next
                // posedge). Actually no - non-blocking means the VRF sees
                // the NEW re value because combinational VRF reads
                // vrf_re through wires continuously.
                //
                // Wait - vrf_re_0 is a reg set with <=. The value becomes
                // visible AFTER this posedge. So during S_READ's posedge,
                // the VRF still sees the OLD value (0). The NEW value (1)
                // is visible between S_READ and S_EXEC posedges.
                //
                // So in S_EXEC, the VRF has re=1 (set in S_READ) and
                // rdata is valid! We capture it here.
                S_EXEC: begin
                    // Capture data while re is still high from S_READ
                    op_a <= vrf_rdata_0;
                    op_b <= vrf_rdata_1;
                    op_c <= needs_vd ? vrf_rdata_2 : {WORD_W{1'b0}};

                    // De-assert reads
                    vrf_re_0 <= 0; vrf_re_1 <= 0; vrf_re_2 <= 0;
                    vrf_we <= 0;

                    // Compute using BLOCKING reads of the input data
                    for (idx = 0; idx < N_FU; idx = idx + 1) begin
                        a_el = vrf_rdata_0[idx*ELEN +: ELEN];
                        b_el = vrf_rdata_1[idx*ELEN +: ELEN];
                        c_el = needs_vd ? vrf_rdata_2[idx*ELEN +: ELEN] : 0;
                        case (op_r)
                            2'b00: result[idx*ELEN +: ELEN] = a_el + b_el;
                            2'b01: result[idx*ELEN +: ELEN] = a_el * b_el;
                            2'b10: result[idx*ELEN +: ELEN] = c_el + a_el * b_el;
                            2'b11: result[idx*ELEN +: ELEN] = a_el - b_el;
                        endcase
                    end
                    state <= S_WB;
                end

                S_WB: begin
                    vrf_re_0 <= 0; vrf_re_1 <= 0; vrf_re_2 <= 0;
                    vrf_waddr <= {vd_r, word_cnt[1:0]};
                    vrf_wdata <= result;
                    vrf_we    <= 1;
                    state     <= S_NEXT;
                end

                S_NEXT: begin
                    vrf_re_0 <= 0; vrf_re_1 <= 0; vrf_re_2 <= 0; vrf_we <= 0;
                    if (word_cnt == WORDS_PER_VREG - 1) begin
                        done  <= 1;
                        state <= S_IDLE;
                    end else begin
                        word_cnt <= word_cnt + 1;
                        state    <= S_READ;
                    end
                end

                default: begin
                    vrf_re_0 <= 0; vrf_re_1 <= 0; vrf_re_2 <= 0; vrf_we <= 0;
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule