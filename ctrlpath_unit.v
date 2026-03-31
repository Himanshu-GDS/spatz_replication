`timescale 1ns / 1ps
module ctrlpath_unit(
    input [6:0] opcode,
    input stall,
    output reg reg_write,
    output reg [1:0] imm_sel,
    output reg alu_src,
    output reg [1:0] alu_op,
    output reg mem_read,
    output reg mem_write,
    output reg mem_to_reg,
    output reg branch,
    output reg jump,
    output reg jalr,
    output reg vector_cmd    // NEW: signals a vector instruction
);

always @ (opcode, stall) begin
    if (stall) begin
        reg_write   = 0;
        imm_sel     = 2'b00;
        alu_src     = 0;
        alu_op      = 2'b00;
        mem_read    = 0;
        mem_write   = 0;
        mem_to_reg  = 0;
        branch      = 0;
        jump        = 0;
        jalr        = 0;
        vector_cmd  = 0;
    end
    else begin
        // Default
        vector_cmd = 0;

        // R-Type
        if (opcode == 7'b0110011) begin
            reg_write = 1; imm_sel = 2'bxx; alu_src = 0; alu_op = 2'b10;
            mem_read = 0; mem_write = 0; mem_to_reg = 0;
            branch = 0; jump = 0; jalr = 0; vector_cmd = 0;
        end
        // I-Type ALU
        else if (opcode == 7'b0010011) begin
            reg_write = 1; imm_sel = 2'b00; alu_src = 1; alu_op = 2'b01;
            mem_read = 0; mem_write = 0; mem_to_reg = 0;
            branch = 0; jump = 0; jalr = 0; vector_cmd = 0;
        end
        // Load
        else if (opcode == 7'b0000011) begin
            reg_write = 1; imm_sel = 2'b00; alu_src = 1; alu_op = 2'b00;
            mem_read = 1; mem_write = 0; mem_to_reg = 1;
            branch = 0; jump = 0; jalr = 0; vector_cmd = 0;
        end
        // Store
        else if (opcode == 7'b0100011) begin
            reg_write = 0; imm_sel = 2'b01; alu_src = 1; alu_op = 2'b00;
            mem_read = 0; mem_write = 1; mem_to_reg = 0;
            branch = 0; jump = 0; jalr = 0; vector_cmd = 0;
        end
        // B-Type
        else if (opcode == 7'b1100011) begin
            reg_write = 0; imm_sel = 2'b10; alu_src = 0; alu_op = 2'b11;
            mem_read = 0; mem_write = 0; mem_to_reg = 1'bx;
            branch = 1; jump = 0; jalr = 0; vector_cmd = 0;
        end
        // JAL
        else if (opcode == 7'b1101111) begin
            reg_write = 1; imm_sel = 2'b11; alu_src = 1'bx; alu_op = 2'bxx;
            mem_read = 0; mem_write = 0; mem_to_reg = 1'bx;
            branch = 0; jump = 1; jalr = 0; vector_cmd = 0;
        end
        // JALR
        else if (opcode == 7'b1100111) begin
            reg_write = 1; imm_sel = 2'b00; alu_src = 1'bx; alu_op = 2'bxx;
            mem_read = 0; mem_write = 0; mem_to_reg = 1'bx;
            branch = 0; jump = 1; jalr = 1; vector_cmd = 0;
        end
        // VECTOR: opcode = 1010111 (V-extension arithmetic)
        // and VECTOR LOAD/STORE: opcode = 0000111 (VL) / 0100111 (VS)
        else if (opcode == 7'b1010111 || opcode == 7'b0000111 || opcode == 7'b0100111) begin
            reg_write = 0; imm_sel = 2'b00; alu_src = 0; alu_op = 2'b00;
            mem_read = 0; mem_write = 0; mem_to_reg = 0;
            branch = 0; jump = 0; jalr = 0;
            vector_cmd = 1;   // Signal: this is a vector instruction
        end
        // EBREAK (signals done)
        else if (opcode == 7'b1110011) begin
            reg_write = 0; imm_sel = 2'b00; alu_src = 0; alu_op = 2'b00;
            mem_read = 0; mem_write = 0; mem_to_reg = 0;
            branch = 0; jump = 0; jalr = 0; vector_cmd = 0;
        end
        // Default
        else begin
            reg_write = 0; imm_sel = 2'b00; alu_src = 0; alu_op = 2'b00;
            mem_read = 0; mem_write = 0; mem_to_reg = 0;
            branch = 0; jump = 0; jalr = 0; vector_cmd = 0;
        end
    end
end

endmodule