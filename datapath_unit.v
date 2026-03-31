`timescale 1ns / 1ps
module datapath_unit(
    input rst, clk,
    input if_id_reg_write, if_id_alu_src,
    input if_id_mem_read, if_id_mem_write,
    input if_id_mem_to_reg,
    input [1:0] if_id_imm_sel,
    input [1:0] if_id_alu_op,
    input [1:0] forward_rs1, forward_rs2,
    input if_id_branch, if_id_jump, if_id_jalr,
    input id_ex_pc_src,
    input stall, flush,
    output [6:0] if_id_opcode,
    output [2:0] id_ex_f3,
    output id_ex_branch, id_ex_jump, id_ex_jalr,
    output id_ex_zero, id_ex_neg,
    output [4:0] id_ex_rs1, id_ex_rs2,
    output [4:0] ex_mem_rd, mem_wb_rd,
    output ex_mem_reg_write, mem_wb_reg_write,
    output id_ex_mem_read,
    output [4:0] id_ex_rd,
    output [4:0] if_id_rs1, if_id_rs2,
    // NEW outputs for vector bridge
    output [31:0] if_id_instr_out,
    output [31:0] if_id_reg_data1_out
);

    wire id_ex_reg_write, id_ex_alu_src, id_ex_mem_write, id_ex_mem_to_reg;
    wire ex_mem_mem_read, ex_mem_mem_write, ex_mem_mem_to_reg, ex_mem_jump;
    wire mem_wb_mem_to_reg, mem_wb_jump;
    wire [1:0] id_ex_alu_op;
    wire [2:0] if_id_f3;
    wire [3:0] id_ex_alu_ctrl;
    wire [4:0] if_id_rd;
    wire [6:0] if_id_f7, id_ex_f7;
    wire [31:0] pc_in, pc_out, pc_plus4, pc_in_branch;
    wire [31:0] instr;
    wire [31:0] mem_wb_write_data, if_id_reg_data1, if_id_reg_data2, if_id_imm_val;
    wire [31:0] id_ex_alu_src_mux_out, id_ex_alu_result, zero_flag, read_data;
    wire [31:0] mem_wb_rd_mux_in0, id_ex_jump_src_mux_out, id_ex_branch_addr;
    wire [31:0] if_id_pc_out, if_id_instr, id_ex_reg_data1, id_ex_reg_data2;
    wire [31:0] id_ex_imm_val, id_ex_pc_out, id_ex_pc_plus4;
    wire [31:0] ex_mem_pc_out, ex_mem_reg_data2, ex_mem_alu_result, ex_mem_read_data;
    wire [31:0] mem_wb_pc_out, mem_wb_read_data, mem_wb_alu_result, mem_wb_pc_plus4;
    wire [31:0] fd_mux_rs1_reg_data1, fd_mux_rs2_reg_data2;

    // Expose to vector bridge
    assign if_id_instr_out     = if_id_instr;
    assign if_id_reg_data1_out = if_id_reg_data1;

    // IF Stage
    mux_21 pc_mux (pc_plus4, id_ex_branch_addr, id_ex_pc_src, pc_in);
    pc pc1 (rst, clk, stall, pc_in, pc_out);
    adder_32bit pc_plus4_adder1 (pc_out, 4, pc_plus4);
    instr_mem i_mem (rst, pc_out, instr);

    // IF/ID Pipeline Register
    IF_ID_Reg if_id_reg (
        clk, stall, flush,
        pc_out, instr,
        if_id_pc_out, if_id_instr, if_id_opcode,
        if_id_rs1, if_id_rs2, if_id_rd,
        if_id_f7, if_id_f3
    );

    // Register File
    reg_file rf1 (
        rst, clk,
        if_id_rs1, if_id_rs2,
        mem_wb_rd, mem_wb_write_data, mem_wb_reg_write,
        if_id_reg_data1, if_id_reg_data2
    );

    // Immediate generator
    imm_gen imm_gen1 (if_id_instr, if_id_imm_sel, if_id_imm_val);

    // ID/EX Pipeline Register
        ID_EX_Reg id_ex_reg (
        clk, stall, flush,
        if_id_pc_out, if_id_rs1, if_id_rs2, if_id_rd, if_id_f7, if_id_f3,
        if_id_reg_data1, if_id_reg_data2, if_id_imm_val,
        if_id_reg_write, if_id_alu_src, if_id_alu_op,
        if_id_mem_read, if_id_mem_write, if_id_mem_to_reg,
        if_id_branch, if_id_jump, if_id_jalr,
        id_ex_reg_data1, id_ex_reg_data2, id_ex_imm_val, id_ex_pc_out,
        id_ex_rs1, id_ex_rs2, id_ex_rd, id_ex_f7, id_ex_f3,
        id_ex_reg_write, id_ex_alu_src, id_ex_alu_op,
        id_ex_mem_read, id_ex_mem_write, id_ex_mem_to_reg,
        id_ex_branch, id_ex_jump, id_ex_jalr
    );

    // EX Stage
    mux_21 jump_src_mux (id_ex_pc_out, fd_mux_rs1_reg_data1, id_ex_jalr, id_ex_jump_src_mux_out);
    adder_32bit branch_adder (id_ex_jump_src_mux_out, id_ex_imm_val, id_ex_branch_addr);
    mux_41 fd_mux_rs1 (id_ex_reg_data1, ex_mem_alu_result, mem_wb_rd_mux_in0, 0, forward_rs1, fd_mux_rs1_reg_data1);
    mux_41 fd_mux_rs2 (id_ex_reg_data2, ex_mem_alu_result, mem_wb_rd_mux_in0, 0, forward_rs2, fd_mux_rs2_reg_data2);
    mux_21 alu_src_mux (fd_mux_rs2_reg_data2, id_ex_imm_val, id_ex_alu_src, id_ex_alu_src_mux_out);
    alu_ctrl_unit alu_control (id_ex_alu_op, id_ex_f3, id_ex_f7, id_ex_alu_ctrl);
    alu alu1 (fd_mux_rs1_reg_data1, id_ex_alu_src_mux_out, id_ex_alu_ctrl, id_ex_alu_result, id_ex_zero, id_ex_neg);

    // EX/MEM Pipeline Register
    EX_MEM_Reg ex_mem_reg (
        clk, id_ex_pc_out, id_ex_rd, fd_mux_rs2_reg_data2, id_ex_alu_result,
        id_ex_reg_write, id_ex_mem_read, id_ex_mem_write, id_ex_mem_to_reg, id_ex_jump,
        ex_mem_pc_out, ex_mem_rd, ex_mem_reg_data2, ex_mem_alu_result,
        ex_mem_reg_write, ex_mem_mem_read, ex_mem_mem_write, ex_mem_mem_to_reg, ex_mem_jump
    );

    // MEM Stage
    data_mem d_mem (rst, clk, ex_mem_alu_result, ex_mem_reg_data2, ex_mem_mem_read, ex_mem_mem_write, ex_mem_read_data);

    // MEM/WB Pipeline Register
    MEM_WB_Reg mem_wb_reg (
        clk, ex_mem_pc_out, ex_mem_rd, ex_mem_read_data, ex_mem_alu_result,
        ex_mem_reg_write, ex_mem_mem_to_reg, ex_mem_jump,
        mem_wb_pc_out, mem_wb_rd, mem_wb_read_data, mem_wb_alu_result,
        mem_wb_reg_write, mem_wb_mem_to_reg, mem_wb_jump
    );

    // WB Stage
    mux_21 mem_to_reg_mux (mem_wb_alu_result, mem_wb_read_data, mem_wb_mem_to_reg, mem_wb_rd_mux_in0);
    adder_32bit pc_plus4_adder2 (mem_wb_pc_out, 4, mem_wb_pc_plus4);
    mux_21 rd_mux (mem_wb_rd_mux_in0, mem_wb_pc_plus4, mem_wb_jump, mem_wb_write_data);

endmodule