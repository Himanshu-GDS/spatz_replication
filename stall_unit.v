`timescale 1ns / 1ps
//======================================================
// Module Name: stall_unit
// Description:
//   This module detects data hazards between pipeline
//   stages in a pipelined processor and decides whether
//   to stall the pipeline.
//
//   Specifically, it detects "load-use hazards" — cases
//   where an instruction in the ID stage needs data that
//   is still being loaded from memory by the EX stage.
//
// Inputs:
//   - rst            : Reset signal (active high)
//   - if_id_mem_read : Indicates if the instruction in IF/ID
//                      stage is a load instruction
//   - id_ex_mem_read : Indicates if the instruction in ID/EX
//                      stage is a load instruction
//   - id_ex_rd       : Destination register of ID/EX instruction
//   - if_id_rs1      : Source register 1 of IF/ID instruction
//   - if_id_rs2      : Source register 2 of IF/ID instruction
//
// Output:
//   - stall : 1 ? Stall pipeline; 0 ? No stall
//
//======================================================

module stall_unit(
    input rst,               // Reset signal
    input if_id_mem_read,    // Is IF/ID instruction a load?
    input id_ex_mem_read,    // Is ID/EX instruction a load?
    input [4:0] id_ex_rd,    // Destination register in ID/EX stage
    input [4:0] if_id_rs1,   // Source register 1 in IF/ID stage
    input [4:0] if_id_rs2,   // Source register 2 in IF/ID stage
    output reg stall         // Stall control output
);

always @(*) begin
    // Case 1: On reset ? No stall
    if (rst) begin
        stall = 0;
    end
    // Case 2: Possible load-use hazard
    // Conditions:
    //   - Current instruction in EX stage is a load
    //   - Destination register (id_ex_rd) is not zero (avoid x0)
    //   - Destination matches either source register of IF/ID stage
    else if ((id_ex_mem_read == 1) && (id_ex_rd != 0) &&
            ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2))) begin
        
        // Special case: If IF/ID is also a load and the conflict
        // is with RS2 ? Don't stall
        if ((if_id_mem_read == 1) && (id_ex_rd == if_id_rs2))
            stall = 0;
        else
            stall = 1; // Stall pipeline to avoid hazard
    end
    // Case 3: No hazard ? No stall
    else begin
        stall = 0;
    end
end

endmodule
