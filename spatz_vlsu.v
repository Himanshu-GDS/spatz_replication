`timescale 1ns / 1ps
module spatz_vlsu #(
    parameter ELEN       = 32,
    parameter N_FU       = 4,
    parameter VLEN       = 512,
    parameter SPM_ADDR_W = 17
)(
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    cmd_valid,
    output wire                    cmd_ready,
    input  wire                    cmd_is_load,
    input  wire [4:0]              cmd_vreg,
    input  wire [SPM_ADDR_W-1:0]  cmd_spm_addr,
    output reg  [6:0]              vrf_raddr,
    output reg                     vrf_re,
    input  wire [N_FU*ELEN-1:0]   vrf_rdata,
    input  wire                    vrf_rvalid,
    output reg  [6:0]              vrf_waddr,
    output reg  [N_FU*ELEN-1:0]   vrf_wdata,
    output reg                     vrf_we,
    input  wire                    vrf_wvalid,
    output reg  [SPM_ADDR_W-1:0]  spm_addr,
    output reg  [N_FU*ELEN-1:0]   spm_wdata,
    output reg                     spm_we,
    output reg                     spm_req,
    input  wire [N_FU*ELEN-1:0]   spm_rdata,
    input  wire                    spm_gnt,
    output wire                    busy,
    output reg                     done
);

    localparam WORD_W         = N_FU * ELEN;
    localparam WORD_BYTES     = WORD_W / 8;
    localparam WORDS_PER_VREG = VLEN / WORD_W;

    localparam S_IDLE     = 4'd0;
    localparam S_LD_REQ   = 4'd1;
    localparam S_LD_WAIT  = 4'd2;
    localparam S_LD_WB    = 4'd3;
    localparam S_ST_RD    = 4'd4;
    localparam S_ST_CAP   = 4'd5;
    localparam S_ST_WR    = 4'd6;

    reg [3:0] state;
    reg       is_load_r;
    reg [4:0] vreg_r;
    reg [SPM_ADDR_W-1:0] base_r;
    reg [2:0] word_cnt;
    reg [WORD_W-1:0] st_data;  // Captured store data

    assign busy      = (state != S_IDLE);
    assign cmd_ready = (state == S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            word_cnt <= 0;
            vrf_re <= 0; vrf_we <= 0;
            spm_req <= 0; spm_we <= 0;
            done <= 0;
        end else begin
            done <= 0; vrf_we <= 0; vrf_re <= 0;
            spm_req <= 0; spm_we <= 0;

            case (state)
                S_IDLE: if (cmd_valid) begin
                    is_load_r <= cmd_is_load;
                    vreg_r    <= cmd_vreg;
                    base_r    <= cmd_spm_addr;
                    word_cnt  <= 0;
                    state     <= cmd_is_load ? S_LD_REQ : S_ST_RD;
                end

                // ========== LOAD: SPM ? VRF ==========

                S_LD_REQ: begin
                    spm_addr <= base_r + word_cnt * WORD_BYTES;
                    spm_req  <= 1;
                    spm_we   <= 0;
                    state    <= S_LD_WAIT;
                end

                S_LD_WAIT: begin
                    // 1 cycle for SPM registered read
                    state <= S_LD_WB;
                end

                S_LD_WB: begin
                    vrf_waddr <= {vreg_r, word_cnt[1:0]};
                    vrf_wdata <= spm_rdata;
                    vrf_we    <= 1;
                    if (word_cnt == WORDS_PER_VREG - 1) begin
                        done  <= 1;
                        state <= S_IDLE;
                    end else begin
                        word_cnt <= word_cnt + 1;
                        state    <= S_LD_REQ;
                    end
                end

                // ========== STORE: VRF ? SPM ==========
                // With combinational VRF: assert re, capture data same cycle

                S_ST_RD: begin
                    vrf_raddr <= {vreg_r, word_cnt[1:0]};
                    vrf_re    <= 1;
                    state     <= S_ST_CAP;
                end

                S_ST_CAP: begin
                    // Combinational VRF: rvalid was high last cycle when re was high
                    // But data comes through combinational path - need to check
                    // With combinational VRF, rvalid goes high same cycle as re
                    // But re was set via non-blocking assign, so VRF sees it next cycle
                    // So we need to keep re asserted and capture when rvalid is high
                    if (vrf_rvalid) begin
                        st_data <= vrf_rdata;
                        state   <= S_ST_WR;
                    end else begin
                        // Re-assert read (VRF will see re_i next cycle)
                        vrf_raddr <= {vreg_r, word_cnt[1:0]};
                        vrf_re    <= 1;
                    end
                end

                S_ST_WR: begin
                    spm_addr  <= base_r + word_cnt * WORD_BYTES;
                    spm_wdata <= st_data;
                    spm_we    <= 1;
                    spm_req   <= 1;
                    if (word_cnt == WORDS_PER_VREG - 1) begin
                        done  <= 1;
                        state <= S_IDLE;
                    end else begin
                        word_cnt <= word_cnt + 1;
                        state    <= S_ST_RD;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule