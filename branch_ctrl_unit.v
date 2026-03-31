`timescale 1ns / 1ps   // Sets the simulation time unit to 1ns and time precision to 1ps

// -----------------------------------------------------------------------------
// Module: branch_ctrl_unit
// Purpose: Determines whether the program counter (PC) should branch or jump
//          based on instruction type and ALU comparison results.
// Inputs:
//   - f3   : 3-bit funct3 field from the instruction (determines branch type)
//   - branch : Control signal indicating if current instruction is a branch
//   - jump   : Control signal indicating if current instruction is a jump (JAL/JALR)
//   - zero   : ALU zero flag (1 if operands are equal)
//   - neg    : ALU negative flag (1 if result is negative)
// Outputs:
//   - pc_src : Signal to select the next PC value (1 = take branch/jump, 0 = PC+4)
// -----------------------------------------------------------------------------
module branch_ctrl_unit(
    input  [2:0] f3,      // Branch type selector from instruction (funct3)
    input        branch,  // Indicates current instruction is a branch
    input        jump,    // Indicates current instruction is a jump
    input        zero,    // ALU zero flag
    input        neg,     // ALU negative flag
    output reg   pc_src   // PC source control (1 = branch/jump, 0 = sequential)
);

// Combinational logic block — triggered whenever inputs change
always @(f3, branch, jump, zero, neg)
begin
    if (branch == 1)  // Branch instruction detected
    begin
        // -------------------------
        // BEQ: Branch if Equal
        // -------------------------
        if (f3 == 3'b000)         
        begin 
            if (zero == 1)        // Operands are equal
                pc_src = 1;       // Take branch
            else
                pc_src = 0;       // Do not branch
        end

        // -------------------------
        // BNE: Branch if Not Equal
        // -------------------------
        else if (f3 == 3'b001)  
        begin 
            if (zero == 0)        // Operands are not equal
                pc_src = 1;       // Take branch
            else
                pc_src = 0;       // Do not branch
        end

        // -------------------------
        // BLT: Branch if Less Than (signed)
        // -------------------------
        else if (f3 == 3'b100)  
        begin 
            if (neg == 1)         // Result is negative ? rs1 < rs2
                pc_src = 1;       // Take branch
            else
                pc_src = 0;       // Do not branch
        end

        // -------------------------
        // BGE: Branch if Greater or Equal (signed)
        // -------------------------
        else if (f3 == 3'b101)  
        begin 
            if (neg == 0)         // Result is non-negative ? rs1 >= rs2
                pc_src = 1;       // Take branch
            else
                pc_src = 0;       // Do not branch
        end
    end

    // -------------------------
    // Jump instructions (JAL, JALR)
    // -------------------------
    else if (jump == 1)
    begin
        pc_src = 1;  // Always take jump
    end

    // -------------------------
    // Default case: no branch/jump ? sequential PC
    // -------------------------
    else
    begin
        pc_src = 0;  // Next PC = PC + 4
    end
end

endmodule
