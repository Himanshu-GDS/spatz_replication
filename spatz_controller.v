`timescale 1ns / 1ps
module spatz_controller #(
    parameter SPM_ADDR_W = 17
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
    output reg         vau_cmd_valid,
    input  wire        vau_cmd_ready,
    output reg  [1:0]  vau_cmd_op,
    output reg  [4:0]  vau_cmd_vs1,
    output reg  [4:0]  vau_cmd_vs2,
    output reg  [4:0]  vau_cmd_vd,
    input  wire        vau_busy,
    output reg         vlsu_cmd_valid,
    input  wire        vlsu_cmd_ready,
    output reg         vlsu_cmd_is_load,
    output reg  [4:0]  vlsu_cmd_vreg,
    output reg  [SPM_ADDR_W-1:0] vlsu_cmd_spm_addr,
    input  wire        vlsu_busy,
    input  wire        vlsu_done,
    output reg         vsldu_cmd_valid,
    input  wire        vsldu_cmd_ready,
    output reg  [1:0]  vsldu_cmd_op,
    output reg  [4:0]  vsldu_cmd_vs2,
    output reg  [4:0]  vsldu_cmd_vd,
    input  wire        vsldu_busy,
    output wire        busy,
    output wire        ctrl_stall
);

    // ================================================================
    // Dispatch guard: prevents slot clearing on the cycle of dispatch
    // ================================================================
    reg vlsu_dispatched_this_cycle;
    reg vau_dispatched_this_cycle;
    reg vsldu_dispatched_this_cycle;

    // ================================================================
    // Scoreboard slots
    // ================================================================
    reg        slot_vlsu_active;
    reg [4:0]  slot_vlsu_vreg;
    reg        slot_vlsu_is_load;

    reg        slot_vau_active;
    reg [4:0]  slot_vau_vs1;
    reg [4:0]  slot_vau_vs2;
    reg [4:0]  slot_vau_vd;
    reg        slot_vau_reads_vd;

    reg        slot_vsldu_active;
    reg [4:0]  slot_vsldu_vs2;
    reg [4:0]  slot_vsldu_vd;

    // ================================================================
    // Hazard detection
    // ================================================================
    wire is_load   = (cmd_op == 3'b000);
    wire is_store  = (cmd_op == 3'b001);
    wire is_vlsu   = is_load | is_store;
    wire is_vsldu  = (cmd_op == 3'b010) | (cmd_op == 3'b111);
    wire is_vau    = ~is_vlsu & ~is_vsldu;
    wire is_fma    = (cmd_op == 3'b100);

    wire vlsu_wr = slot_vlsu_active & slot_vlsu_is_load;
    wire vlsu_rd = slot_vlsu_active & ~slot_vlsu_is_load;

    wire h_vlsu_raw = vlsu_wr & (
        (is_vau   & ((slot_vlsu_vreg == cmd_vs1) | (slot_vlsu_vreg == cmd_vs2) |
                      (is_fma & (slot_vlsu_vreg == cmd_vd)))) |
        (is_store & (slot_vlsu_vreg == cmd_vs2)) |
        (is_vsldu & (slot_vlsu_vreg == cmd_vs2))
    );
    wire h_vlsu_waw = vlsu_wr & (
        (is_load  & (slot_vlsu_vreg == cmd_vd)) |
        (is_vau   & (slot_vlsu_vreg == cmd_vd)) |
        (is_vsldu & (slot_vlsu_vreg == cmd_vd))
    );
    wire h_vlsu_war = vlsu_rd & (
        (is_load  & (slot_vlsu_vreg == cmd_vd)) |
        (is_vau   & (slot_vlsu_vreg == cmd_vd)) |
        (is_vsldu & (slot_vlsu_vreg == cmd_vd))
    );
    wire hazard_vlsu = h_vlsu_raw | h_vlsu_waw | h_vlsu_war;

    wire h_vau_raw = slot_vau_active & (
        (is_load  & (slot_vau_vd == cmd_vd)) |
        (is_store & (slot_vau_vd == cmd_vs2)) |
        (is_vau   & ((slot_vau_vd == cmd_vs1) | (slot_vau_vd == cmd_vs2) |
                      (is_fma & (slot_vau_vd == cmd_vd)))) |
        (is_vsldu & (slot_vau_vd == cmd_vs2))
    );
    wire h_vau_waw = slot_vau_active & (
        (is_load  & (slot_vau_vd == cmd_vd)) |
        (is_vau   & (slot_vau_vd == cmd_vd)) |
        (is_vsldu & (slot_vau_vd == cmd_vd))
    );
    wire h_vau_war = slot_vau_active & (
        ((is_load | is_vau | is_vsldu) & (slot_vau_vs1 == cmd_vd)) |
        ((is_load | is_vau | is_vsldu) & (slot_vau_vs2 == cmd_vd)) |
        (slot_vau_reads_vd & (is_load | is_vau | is_vsldu) & (slot_vau_vd == cmd_vd))
    );
    wire hazard_vau = h_vau_raw | h_vau_waw | h_vau_war;

    wire h_vsldu_raw = slot_vsldu_active & (
        (slot_vsldu_vd == cmd_vs1) | (slot_vsldu_vd == cmd_vs2) |
        (is_fma & (slot_vsldu_vd == cmd_vd))
    );
    wire h_vsldu_waw = slot_vsldu_active & (
        (is_load | is_vau | is_vsldu) & (slot_vsldu_vd == cmd_vd)
    );
    wire hazard_vsldu = h_vsldu_raw | h_vsldu_waw;

    wire target_busy = (is_vlsu  & slot_vlsu_active) |
                       (is_vau   & slot_vau_active) |
                       (is_vsldu & slot_vsldu_active);

    wire has_hazard = hazard_vlsu | hazard_vau | hazard_vsldu | target_busy;

    // ================================================================
    // Stall path hazard check
    // ================================================================
    localparam S_IDLE  = 2'd0;
    localparam S_STALL = 2'd1;

    reg [1:0] state;
    reg [2:0] stall_op;
    reg [4:0] stall_vs1, stall_vs2, stall_vd;
    reg [SPM_ADDR_W-1:0] stall_addr;
    reg stall_r;

    wire st_is_load  = (stall_op == 3'b000);
    wire st_is_store = (stall_op == 3'b001);
    wire st_is_vlsu  = st_is_load | st_is_store;
    wire st_is_vsldu = (stall_op == 3'b010) | (stall_op == 3'b111);
    wire st_is_vau   = ~st_is_vlsu & ~st_is_vsldu;
    wire st_is_fma   = (stall_op == 3'b100);

    wire sh_vlsu_raw = vlsu_wr & (
        (st_is_vau   & ((slot_vlsu_vreg == stall_vs1) | (slot_vlsu_vreg == stall_vs2) |
                         (st_is_fma & (slot_vlsu_vreg == stall_vd)))) |
        (st_is_store & (slot_vlsu_vreg == stall_vs2)) |
        (st_is_vsldu & (slot_vlsu_vreg == stall_vs2))
    );
    wire sh_vlsu_waw = vlsu_wr & (
        (st_is_load | st_is_vau | st_is_vsldu) & (slot_vlsu_vreg == stall_vd)
    );
    wire sh_vlsu_war = vlsu_rd & (
        (st_is_load | st_is_vau | st_is_vsldu) & (slot_vlsu_vreg == stall_vd)
    );
    wire sh_vlsu = sh_vlsu_raw | sh_vlsu_waw | sh_vlsu_war;

    wire sh_vau_raw = slot_vau_active & (
        (st_is_load  & (slot_vau_vd == stall_vd)) |
        (st_is_store & (slot_vau_vd == stall_vs2)) |
        (st_is_vau   & ((slot_vau_vd == stall_vs1) | (slot_vau_vd == stall_vs2) |
                         (st_is_fma & (slot_vau_vd == stall_vd)))) |
        (st_is_vsldu & (slot_vau_vd == stall_vs2))
    );
    wire sh_vau_waw = slot_vau_active & (
        (st_is_load | st_is_vau | st_is_vsldu) & (slot_vau_vd == stall_vd)
    );
    wire sh_vau_war = slot_vau_active & (
        ((st_is_load | st_is_vau | st_is_vsldu) & (slot_vau_vs1 == stall_vd)) |
        ((st_is_load | st_is_vau | st_is_vsldu) & (slot_vau_vs2 == stall_vd)) |
        (slot_vau_reads_vd & (st_is_load | st_is_vau | st_is_vsldu) & (slot_vau_vd == stall_vd))
    );
    wire sh_vau = sh_vau_raw | sh_vau_waw | sh_vau_war;

    wire sh_vsldu_raw = slot_vsldu_active & (
        (slot_vsldu_vd == stall_vs1) | (slot_vsldu_vd == stall_vs2) |
        (st_is_fma & (slot_vsldu_vd == stall_vd))
    );
    wire sh_vsldu_waw = slot_vsldu_active & (
        (st_is_load | st_is_vau | st_is_vsldu) & (slot_vsldu_vd == stall_vd)
    );
    wire sh_vsldu = sh_vsldu_raw | sh_vsldu_waw;

    wire stall_target_busy = (st_is_vlsu  & slot_vlsu_active) |
                             (st_is_vau   & slot_vau_active) |
                             (st_is_vsldu & slot_vsldu_active);

    wire stall_has_hazard = sh_vlsu | sh_vau | sh_vsldu | stall_target_busy;

    // ================================================================
    // Busy / ready
    // ================================================================
    assign busy       = (state == S_STALL) | slot_vlsu_active | slot_vau_active | slot_vsldu_active;
    assign cmd_ready  = (state == S_IDLE);
    assign ctrl_stall = stall_r;

    // ================================================================
    // Main FSM
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            vau_cmd_valid <= 0; vlsu_cmd_valid <= 0; vsldu_cmd_valid <= 0;
            slot_vlsu_active  <= 0;
            slot_vau_active   <= 0;
            slot_vsldu_active <= 0;
            stall_r <= 0;
            vlsu_dispatched_this_cycle  <= 0;
            vau_dispatched_this_cycle   <= 0;
            vsldu_dispatched_this_cycle <= 0;
        end else begin
            vau_cmd_valid <= 0; vlsu_cmd_valid <= 0; vsldu_cmd_valid <= 0;
            stall_r <= 0;

            // Reset dispatch guards
            vlsu_dispatched_this_cycle  <= 0;
            vau_dispatched_this_cycle   <= 0;
            vsldu_dispatched_this_cycle <= 0;

            // ---- Clear slots when EU is no longer busy ----
            // Guard: don't clear on the same cycle we dispatched
            if (slot_vlsu_active && !vlsu_busy && !vlsu_dispatched_this_cycle)
                slot_vlsu_active <= 0;
            if (slot_vau_active && !vau_busy && !vau_dispatched_this_cycle)
                slot_vau_active <= 0;
            if (slot_vsldu_active && !vsldu_busy && !vsldu_dispatched_this_cycle)
                slot_vsldu_active <= 0;

            case (state)
                S_IDLE: begin
                    if (cmd_valid) begin
                        if (!has_hazard) begin
                            if (is_vlsu) begin
                                vlsu_cmd_valid    <= 1;
                                vlsu_cmd_is_load  <= is_load;
                                vlsu_cmd_vreg     <= is_load ? cmd_vd : cmd_vs2;
                                vlsu_cmd_spm_addr <= cmd_spm_addr;
                                slot_vlsu_active  <= 1;
                                slot_vlsu_vreg    <= is_load ? cmd_vd : cmd_vs2;
                                slot_vlsu_is_load <= is_load;
                                vlsu_dispatched_this_cycle <= 1;
                            end else if (is_vsldu) begin
                                vsldu_cmd_valid   <= 1;
                                vsldu_cmd_op      <= (cmd_op == 3'b010) ? 2'b10 :
                                                     (cmd_op == 3'b111) ? 2'b00 : 2'b01;
                                vsldu_cmd_vs2     <= cmd_vs2;
                                vsldu_cmd_vd      <= cmd_vd;
                                slot_vsldu_active <= 1;
                                slot_vsldu_vs2    <= cmd_vs2;
                                slot_vsldu_vd     <= cmd_vd;
                                vsldu_dispatched_this_cycle <= 1;
                            end else begin
                                vau_cmd_valid <= 1;
                                case (cmd_op)
                                    3'b011: vau_cmd_op <= 2'b00;
                                    3'b100: vau_cmd_op <= 2'b10;
                                    3'b101: vau_cmd_op <= 2'b01;
                                    3'b110: vau_cmd_op <= 2'b11;
                                    default: vau_cmd_op <= 2'b00;
                                endcase
                                vau_cmd_vs1       <= cmd_vs1;
                                vau_cmd_vs2       <= cmd_vs2;
                                vau_cmd_vd        <= cmd_vd;
                                slot_vau_active   <= 1;
                                slot_vau_vs1      <= cmd_vs1;
                                slot_vau_vs2      <= cmd_vs2;
                                slot_vau_vd       <= cmd_vd;
                                slot_vau_reads_vd <= is_fma;
                                vau_dispatched_this_cycle <= 1;
                            end
                            state <= S_IDLE;
                        end else begin
                            stall_op   <= cmd_op;
                            stall_vs1  <= cmd_vs1;
                            stall_vs2  <= cmd_vs2;
                            stall_vd   <= cmd_vd;
                            stall_addr <= cmd_spm_addr;
                            stall_r    <= 1;
                            state      <= S_STALL;
                        end
                    end
                end

                S_STALL: begin
                    if (!stall_has_hazard) begin
                        if (st_is_vlsu) begin
                            vlsu_cmd_valid    <= 1;
                            vlsu_cmd_is_load  <= st_is_load;
                            vlsu_cmd_vreg     <= st_is_load ? stall_vd : stall_vs2;
                            vlsu_cmd_spm_addr <= stall_addr;
                            slot_vlsu_active  <= 1;
                            slot_vlsu_vreg    <= st_is_load ? stall_vd : stall_vs2;
                            slot_vlsu_is_load <= st_is_load;
                            vlsu_dispatched_this_cycle <= 1;
                        end else if (st_is_vsldu) begin
                            vsldu_cmd_valid   <= 1;
                            vsldu_cmd_op      <= (stall_op == 3'b010) ? 2'b10 :
                                                 (stall_op == 3'b111) ? 2'b00 : 2'b01;
                            vsldu_cmd_vs2     <= stall_vs2;
                            vsldu_cmd_vd      <= stall_vd;
                            slot_vsldu_active <= 1;
                            slot_vsldu_vs2    <= stall_vs2;
                            slot_vsldu_vd     <= stall_vd;
                            vsldu_dispatched_this_cycle <= 1;
                        end else begin
                            vau_cmd_valid <= 1;
                            case (stall_op)
                                3'b011: vau_cmd_op <= 2'b00;
                                3'b100: vau_cmd_op <= 2'b10;
                                3'b101: vau_cmd_op <= 2'b01;
                                3'b110: vau_cmd_op <= 2'b11;
                                default: vau_cmd_op <= 2'b00;
                            endcase
                            vau_cmd_vs1       <= stall_vs1;
                            vau_cmd_vs2       <= stall_vs2;
                            vau_cmd_vd        <= stall_vd;
                            slot_vau_active   <= 1;
                            slot_vau_vs1      <= stall_vs1;
                            slot_vau_vs2      <= stall_vs2;
                            slot_vau_vd       <= stall_vd;
                            slot_vau_reads_vd <= st_is_fma;
                            vau_dispatched_this_cycle <= 1;
                        end
                        state <= S_IDLE;
                    end else begin
                        stall_r <= 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule