`timescale 1ns / 1ps
module riscv_top(
    input         rst,
    input         clk,
    input         vec_stall,

    output        vec_cmd_valid,
    output [31:0] vec_instr,
    output [31:0] vec_rs1_data,

    output        done
);

    wire id_ex_zero, id_ex_neg;
    wire if_id_reg_write, if_id_alu_src;
    wire if_id_mem_read, if_id_mem_write, if_id_mem_to_reg;
    wire if_id_branch, if_id_jump, if_id_jalr;
    wire if_id_vector_cmd;
    wire id_ex_pc_src, id_ex_branch, id_ex_jump, id_ex_jalr;
    wire ex_mem_reg_write, mem_wb_reg_write;
    wire id_ex_mem_read;
    wire stall_hazard;
    wire flush;
    wire [1:0] if_id_imm_sel, if_id_alu_op;
    wire [1:0] forward_rs1, forward_rs2;
    wire [2:0] id_ex_f3;
    wire [4:0] id_ex_rs1, id_ex_rs2;
    wire [4:0] ex_mem_rd, mem_wb_rd;
    wire [4:0] id_ex_rd;
    wire [4:0] if_id_rs1, if_id_rs2;
    wire [6:0] if_id_opcode;
    wire [31:0] instr;

    assign flush = id_ex_pc_src;

    wire [31:0] if_id_instr_out;
    wire [31:0] if_id_reg_data1_out;

    // Control path: only hazard stall
    wire ctrl_stall = stall_hazard;

    // ================================================================
    // Vector dispatch with 1-cycle settle time
    //
    // When a vector instruction first appears in IF/ID, the non-blocking
    // assignments mean if_id_rs1/if_id_instr aren't valid yet on the
    // same posedge. We need to wait 1 cycle for IF/ID to settle.
    //
    // vec_seen: set when we first see a vector instruction in IF/ID
    //           (stalls the pipeline but doesn't dispatch yet)
    // vec_cmd_sent: set when we actually dispatch (1 cycle after vec_seen)
    //
    // Cycle 0: pipeline running, new vec instr latched into IF/ID (<=)
    //          vec_hold=1 stalls pipeline. No dispatch yet.
    // Cycle 1: IF/ID has settled values. vec_dispatching=1, dispatch!
    //          vec_cmd_sent=1, pipeline stays stalled.
    // Cycle 2+: vec_stall=1 from bridge, vec_cmd_sent=1, no re-dispatch.
    // ================================================================

    reg vec_cmd_sent;
    reg vec_seen;

    // vec_hold: stall pipeline when vector instruction detected but
    // not yet dispatched (waiting for IF/ID to settle)
    wire vec_hold = if_id_vector_cmd & ~vec_cmd_sent & ~vec_stall;

    // vec_dispatching: fire on the second cycle (vec_seen already set)
    wire vec_dispatching = vec_seen & ~vec_cmd_sent & ~vec_stall & ~stall_hazard;

    wire stall = stall_hazard | vec_stall | vec_hold;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            vec_cmd_sent <= 0;
            vec_seen     <= 0;
        end else begin
            if (vec_dispatching) begin
                vec_cmd_sent <= 1;
                vec_seen     <= 0;
            end else if (vec_hold & ~vec_seen) begin
                // First cycle: just saw vector instr, stall and wait
                vec_seen <= 1;
            end else if (!stall & !vec_stall) begin
                // Pipeline advancing with new instruction
                vec_cmd_sent <= 0;
                vec_seen     <= 0;
            end
        end
    end

    assign vec_cmd_valid = vec_dispatching;
    assign vec_instr     = if_id_instr_out;
    assign vec_rs1_data  = if_id_reg_data1_out;

    assign done = (if_id_opcode == 7'b1110011);

    datapath_unit dp (
        rst, clk,
        if_id_reg_write, if_id_alu_src, if_id_mem_read, if_id_mem_write,
        if_id_mem_to_reg, if_id_imm_sel, if_id_alu_op,
        forward_rs1, forward_rs2,
        if_id_branch, if_id_jump, if_id_jalr,
        id_ex_pc_src, stall, flush,
        if_id_opcode, id_ex_f3,
        id_ex_branch, id_ex_jump, id_ex_jalr,
        id_ex_zero, id_ex_neg,
        id_ex_rs1, id_ex_rs2,
        ex_mem_rd, mem_wb_rd,
        ex_mem_reg_write, mem_wb_reg_write,
        id_ex_mem_read, id_ex_rd,
        if_id_rs1, if_id_rs2,
        if_id_instr_out,
        if_id_reg_data1_out
    );

    ctrlpath_unit cp (
        if_id_opcode, ctrl_stall,
        if_id_reg_write, if_id_imm_sel, if_id_alu_src, if_id_alu_op,
        if_id_mem_read, if_id_mem_write, if_id_mem_to_reg,
        if_id_branch, if_id_jump, if_id_jalr,
        if_id_vector_cmd
    );

    branch_ctrl_unit bcu (
        id_ex_f3, id_ex_branch, id_ex_jump,
        id_ex_zero, id_ex_neg,
        id_ex_pc_src
    );

    fwd_unit fdu (
        id_ex_rs1, id_ex_rs2,
        ex_mem_rd, mem_wb_rd,
        ex_mem_reg_write, mem_wb_reg_write,
        forward_rs1, forward_rs2
    );

    stall_unit su (
        rst,
        if_id_mem_read, id_ex_mem_read,
        id_ex_rd, if_id_rs1, if_id_rs2,
        stall_hazard
    );

endmodule