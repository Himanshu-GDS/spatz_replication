`timescale 1ns / 1ps
module vector_cmd_bridge (
    input  wire        clk,
    input  wire        rst_n,

    // From RISC-V core
    input  wire        vec_cmd_valid,
    input  wire [31:0] vec_instr,
    input  wire [31:0] vec_rs1_data,

    // To Spatz cluster
    output reg         spatz_cmd_valid,
    input  wire        spatz_cmd_ready,
    output reg  [2:0]  spatz_cmd_op,
    output reg  [4:0]  spatz_cmd_vs1,
    output reg  [4:0]  spatz_cmd_vs2,
    output reg  [4:0]  spatz_cmd_vd,
    output reg  [16:0] spatz_cmd_spm_addr,

    // Stall back to RISC-V core
    output reg         vec_stall,

    // Spatz cluster busy
    input  wire        spatz_busy
);

    // Instruction field extraction
    wire [6:0] opcode = vec_instr[6:0];
    wire [4:0] vd     = vec_instr[11:7];
    wire [2:0] funct3 = vec_instr[14:12];
    wire [4:0] vs1    = vec_instr[19:15];
    wire [4:0] vs2    = vec_instr[24:20];
    wire [5:0] funct6 = vec_instr[31:26];

    // Decode instruction type
    wire is_vload  = (opcode == 7'b0000111);
    wire is_vstore = (opcode == 7'b0100111);
    wire is_varith = (opcode == 7'b1010111);

    wire is_vmul  = is_varith & (funct6 == 6'b100101);
    wire is_vmacc = is_varith & (funct6 == 6'b101101);
    wire is_vadd  = is_varith & (funct6 == 6'b000000);
    wire is_vsub  = is_varith & (funct6 == 6'b000010);

    // State machine
    localparam S_IDLE     = 3'd0;
    localparam S_DECODE   = 3'd1;
    localparam S_DISPATCH = 3'd2;
    localparam S_LAUNCHED = 3'd3;  // Wait for busy to assert
    localparam S_WAIT     = 3'd4;  // Wait for busy to deassert

    reg [2:0] state;
    reg [2:0]  cmd_op_r;
    reg [4:0]  cmd_vs1_r, cmd_vs2_r, cmd_vd_r;
    reg [16:0] cmd_addr_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            spatz_cmd_valid <= 0;
            vec_stall       <= 0;
        end else begin
            spatz_cmd_valid <= 0;

            case (state)
                S_IDLE: begin
                    vec_stall <= 0;
                    if (vec_cmd_valid) begin
                        // Decode and latch
                        if (is_vload) begin
                            cmd_op_r   <= 3'b000;
                            cmd_vs1_r  <= 5'd0;
                            cmd_vs2_r  <= 5'd0;
                            cmd_vd_r   <= vd;
                            cmd_addr_r <= vec_rs1_data[16:0];
                        end
                        else if (is_vstore) begin
                            cmd_op_r   <= 3'b001;
                            cmd_vs1_r  <= 5'd0;
                            cmd_vs2_r  <= vd;
                            cmd_vd_r   <= 5'd0;
                            cmd_addr_r <= vec_rs1_data[16:0];
                        end
                        else if (is_vmul) begin
                            cmd_op_r   <= 3'b101;
                            cmd_vs1_r  <= vs1;
                            cmd_vs2_r  <= vs2;
                            cmd_vd_r   <= vd;
                            cmd_addr_r <= 17'd0;
                        end
                        else if (is_vmacc) begin
                            cmd_op_r   <= 3'b100;
                            cmd_vs1_r  <= vs1;
                            cmd_vs2_r  <= vs2;
                            cmd_vd_r   <= vd;
                            cmd_addr_r <= 17'd0;
                        end
                        else if (is_vadd) begin
                            cmd_op_r   <= 3'b011;
                            cmd_vs1_r  <= vs1;
                            cmd_vs2_r  <= vs2;
                            cmd_vd_r   <= vd;
                            cmd_addr_r <= 17'd0;
                        end
                        else if (is_vsub) begin
                            cmd_op_r   <= 3'b110;
                            cmd_vs1_r  <= vs1;
                            cmd_vs2_r  <= vs2;
                            cmd_vd_r   <= vd;
                            cmd_addr_r <= 17'd0;
                        end

                        vec_stall <= 1;
                        state     <= S_DISPATCH;
                    end
                end

                S_DISPATCH: begin
                    if (spatz_cmd_ready) begin
                        spatz_cmd_valid    <= 1;
                        spatz_cmd_op       <= cmd_op_r;
                        spatz_cmd_vs1      <= cmd_vs1_r;
                        spatz_cmd_vs2      <= cmd_vs2_r;
                        spatz_cmd_vd       <= cmd_vd_r;
                        spatz_cmd_spm_addr <= cmd_addr_r;
                        state              <= S_LAUNCHED;
                    end
                end

                // Wait at least one cycle for Spatz to register the command
                // and for spatz_busy to assert
                S_LAUNCHED: begin
                    // spatz_busy should be high now (or will be next cycle)
                    if (spatz_busy) begin
                        state <= S_WAIT;
                    end
                    // If busy is already low after 1 cycle, the operation
                    // completed instantly (shouldn't happen, but safe)
                end

                // Wait for the operation to complete
                S_WAIT: begin
                    if (!spatz_busy) begin
                        vec_stall <= 0;
                        state     <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule