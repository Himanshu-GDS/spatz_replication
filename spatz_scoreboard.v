// ============================================================================
// Spatz Scoreboard -- Register Dependency Tracking
// Reference: hw/ip/spatz/src/spatz_controller.sv lines 217-484
//
// Tracks which vector registers are being read/written by in-flight
// instructions. Detects:
//   RAW (Read-After-Write): consumer reads a reg being written
//   WAW (Write-After-Write): two writes to same reg
//   WAR (Write-After-Read): writer overwrites reg still being read
//
// The original controller supports NrParallelInstructions=4 in-flight ops.
// We simplify to tracking register-level busy bits.
//
// read_table[vreg]  = 1 if any EU is reading this register
// write_table[vreg] = 1 if any EU is writing this register
// ============================================================================
module spatz_scoreboard #(
    parameter NRVREG = 32
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Check hazards (combinational output)
    input  wire [4:0]              check_vs1,
    input  wire                    check_vs1_valid,
    input  wire [4:0]              check_vs2,
    input  wire                    check_vs2_valid,
    input  wire [4:0]              check_vd,
    input  wire                    check_vd_valid,
    output wire                    hazard,

    // Mark register as busy (when EU starts)
    input  wire                    mark_valid,
    input  wire [4:0]              mark_vs1,
    input  wire                    mark_vs1_rd,     // 1 = reading vs1
    input  wire [4:0]              mark_vs2,
    input  wire                    mark_vs2_rd,     // 1 = reading vs2
    input  wire [4:0]              mark_vd,
    input  wire                    mark_vd_wr,      // 1 = writing vd

    // Clear register busy bits (when EU completes)
    input  wire                    clear_valid,
    input  wire [4:0]              clear_vs1,
    input  wire                    clear_vs1_rd,
    input  wire [4:0]              clear_vs2,
    input  wire                    clear_vs2_rd,
    input  wire [4:0]              clear_vd,
    input  wire                    clear_vd_wr
);

    // Read counter per register (supports multiple simultaneous readers)
    reg [2:0] read_cnt  [0:NRVREG-1];
    // Write flag per register (only one writer at a time)
    reg       write_flag [0:NRVREG-1];

    // Hazard detection (combinational)
    wire raw_vs1 = check_vs1_valid && write_flag[check_vs1];  // vs1 being written
    wire raw_vs2 = check_vs2_valid && write_flag[check_vs2];  // vs2 being written
    wire waw_vd  = check_vd_valid  && write_flag[check_vd];   // vd being written
    wire war_vd  = check_vd_valid  && (read_cnt[check_vd] != 0); // vd being read

    assign hazard = raw_vs1 | raw_vs2 | waw_vd | war_vd;

    // Update scoreboard
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NRVREG; i = i + 1) begin
                read_cnt[i]   <= 3'd0;
                write_flag[i] <= 1'b0;
            end
        end else begin
            // Mark new operation's registers
            if (mark_valid) begin
                if (mark_vs1_rd) read_cnt[mark_vs1]   <= read_cnt[mark_vs1] + 1;
                if (mark_vs2_rd) read_cnt[mark_vs2]   <= read_cnt[mark_vs2] + 1;
                if (mark_vd_wr)  write_flag[mark_vd]  <= 1'b1;
            end

            // Clear completed operation's registers
            if (clear_valid) begin
                if (clear_vs1_rd && read_cnt[clear_vs1] > 0)
                    read_cnt[clear_vs1] <= read_cnt[clear_vs1] - 1;
                if (clear_vs2_rd && read_cnt[clear_vs2] > 0)
                    read_cnt[clear_vs2] <= read_cnt[clear_vs2] - 1;
                if (clear_vd_wr)
                    write_flag[clear_vd] <= 1'b0;
            end
        end
    end

endmodule