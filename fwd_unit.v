`timescale 1ns / 1ps

// ============================================================================
// Module: fwd_unit (Forwarding Unit)
// Purpose: To resolve data hazards in a pipelined processor by forwarding
//          data from later pipeline stages (EX/MEM or MEM/WB) to the EX stage.
// 
// Operation:
//    - If the source register in the EX stage (id_ex_rs1 or id_ex_rs2) matches
//      the destination register in a later stage (EX/MEM or MEM/WB), and that
//      later stage is writing to a register, forward the data to avoid stalls.
//    - Output control signals (forward_rs1, forward_rs2) determine the source:
//         00 -> No forwarding (use normal register file read)
//         01 -> Forward from EX/MEM stage
//         10 -> Forward from MEM/WB stage
// ============================================================================

module fwd_unit(
    input  [4:0] id_ex_rs1,         // Source register 1 in EX stage
    input  [4:0] id_ex_rs2,         // Source register 2 in EX stage
    input  [4:0] ex_mem_rd,         // Destination register in EX/MEM stage
    input  [4:0] mem_wb_rd,         // Destination register in MEM/WB stage
    input        ex_mem_reg_write,  // EX/MEM stage will write to a register
    input        mem_wb_reg_write,  // MEM/WB stage will write to a register
    output reg [1:0] forward_rs1,   // Forwarding control for rs1
    output reg [1:0] forward_rs2    // Forwarding control for rs2
);

always @(
    id_ex_rs1, id_ex_rs2, 
    ex_mem_rd, mem_wb_rd, 
    ex_mem_reg_write, mem_wb_reg_write
)
begin
    // Default: no forwarding (00)
    forward_rs1 = 2'b00;
    forward_rs2 = 2'b00;

    // ---------- Forwarding logic for rs1 ----------
    // Priority 1: Forward from EX/MEM stage if:
    //   - EX/MEM is writing to a register
    //   - Destination register is not x0 (register 0 in RISC-V is always zero)
    //   - Destination register matches rs1 in EX stage
    if ((ex_mem_reg_write == 1) && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs1))
        forward_rs1 = 2'b01;

    // Priority 2: Forward from MEM/WB stage if:
    //   - MEM/WB is writing to a register
    //   - Destination register is not x0
    //   - Destination register matches rs1 in EX stage
    else if ((mem_wb_reg_write == 1) && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs1))
        forward_rs1 = 2'b10;

    // ---------- Forwarding logic for rs2 ----------
    if ((ex_mem_reg_write == 1) && (ex_mem_rd != 0) && (ex_mem_rd == id_ex_rs2))
        forward_rs2 = 2'b01;
    
    else if ((mem_wb_reg_write == 1) && (mem_wb_rd != 0) && (mem_wb_rd == id_ex_rs2))
        forward_rs2 = 2'b10;
end

endmodule
