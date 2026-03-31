`timescale 1ns / 1ps

module mux_21(input [31:0] in0, input [31:0] in1, input sel, output reg [31:0] out);
always@(in0, in1, sel)
begin
    case(sel)
        0 : out = in0;
        1 : out = in1;
        default : out = in0;
    endcase
end
endmodule

`timescale 1ns / 1ps

module pc( input rst, input clk, input stall, input [31:0] pc_in, output reg [31:0] pc_out);
always@( posedge clk)
begin
    if (stall)
        pc_out <= pc_out;
    else begin
        if (rst == 1)
            pc_out = 0;
        else
            pc_out = pc_in;
    end
end
endmodule

`timescale 1ns / 1ps

module adder_32bit(input [31:0] in1, input [31:0] in2, output reg [31:0] sum);
always@(in1, in2) sum = in1 + in2;
endmodule

`timescale 1ns / 10ps

module instr_mem( input rst, input [31:0] read_addr, output reg [31:0] instr);

reg [7:0] instr_mem [127:0];

always@(rst, read_addr)
begin
    if (rst) begin
        // ============================================================
        // MATMUL PROGRAM for Spatz vector processor
        // Computes C = A * B using vector LOAD/MUL/FMA/STORE
        //
        // Register allocation:
        //   x10 (a0) = pointer to A broadcast vectors (increments by 16)
        //   x11 (a1) = pointer to B rows (increments by 16, resets each row)
        //   x12 (a2) = pointer to C result rows (increments by 16)
        //   x13      = row counter i (0..3)
        //   x14      = N = 4 (loop bound)
        //   x15      = column/k counter (0..3)
        //   x16      = B_BASE constant = 0x200 (512)
        //
        // Vector registers:
        //   v1 = A broadcast vector (loaded from SPM)
        //   v2 = B row vector (loaded from SPM)
        //   v3 = accumulator (result)
        //
        // Memory layout (set up by DMA before program runs):
        //   0x000-0x1FF: A broadcast vectors (16 vectors x 16 bytes)
        //   0x200-0x23F: B row vectors (4 vectors x 16 bytes)
        //   0x400-0x43F: C result rows (4 vectors x 16 bytes)
        // ============================================================

        // --- Setup (reordered to avoid data hazards) ---
        // Instr 0 (addr 0x00): addi x10, x0, 0        # a0 = A_BASE = 0
        {instr_mem[3],  instr_mem[2],  instr_mem[1],  instr_mem[0]}  = 32'b000000000000_00000_000_01010_0010011;

        // Instr 1 (addr 0x04): addi x16, x0, 512      # x16 = B_BASE = 0x200
        {instr_mem[7],  instr_mem[6],  instr_mem[5],  instr_mem[4]}  = 32'b001000000000_00000_000_10000_0010011;

        // Instr 2 (addr 0x08): addi x12, x0, 1024     # a2 = C_BASE = 0x400
        {instr_mem[11], instr_mem[10], instr_mem[9],  instr_mem[8]}  = 32'b010000000000_00000_000_01100_0010011;

        // Instr 3 (addr 0x0C): addi x13, x0, 0        # i = 0
        {instr_mem[15], instr_mem[14], instr_mem[13], instr_mem[12]} = 32'b000000000000_00000_000_01101_0010011;

        // Instr 4 (addr 0x10): addi x14, x0, 4        # N = 4
        {instr_mem[19], instr_mem[18], instr_mem[17], instr_mem[16]} = 32'b000000000100_00000_000_01110_0010011;

        // Instr 5 (addr 0x14): addi x11, x16, 0       # a1 = B_BASE (x16 fully written back now)
        {instr_mem[23], instr_mem[22], instr_mem[21], instr_mem[20]} = 32'b000000000000_10000_000_01011_0010011;

        // --- Outer loop: row_loop (addr 0x18) ---
        // Instr 6 (addr 0x18): addi x15, x0, 0        # k = 0
        {instr_mem[27], instr_mem[26], instr_mem[25], instr_mem[24]} = 32'b000000000000_00000_000_01111_0010011;

        // --- Inner loop: k_loop (addr 0x1C) ---
        // Instr 7 (addr 0x1C): vle32.v v1, (x10)
        {instr_mem[31], instr_mem[30], instr_mem[29], instr_mem[28]} = 32'b000000100000_01010_110_00001_0000111;

        // Instr 8 (addr 0x20): vle32.v v2, (x11)
        {instr_mem[35], instr_mem[34], instr_mem[33], instr_mem[32]} = 32'b000000100000_01011_110_00010_0000111;

        // Instr 9 (addr 0x24): bne x15, x0, +12       # if k != 0, skip to FMA (addr 0x30)
        {instr_mem[39], instr_mem[38], instr_mem[37], instr_mem[36]} = 32'b0000000_00000_01111_001_01100_1100011;

        // Instr 10 (addr 0x28): vmul.vv v3, v1, v2
        {instr_mem[43], instr_mem[42], instr_mem[41], instr_mem[40]} = 32'b100101_1_00010_00001_000_00011_1010111;

        // Instr 11 (addr 0x2C): jal x0, +8            # skip FMA (jump to addr 0x34)
        {instr_mem[47], instr_mem[46], instr_mem[45], instr_mem[44]} = 32'b0_0000000100_0_00000000_00000_1101111;

        // Instr 12 (addr 0x30): vmacc.vv v3, v1, v2
        {instr_mem[51], instr_mem[50], instr_mem[49], instr_mem[48]} = 32'b101101_1_00010_00001_000_00011_1010111;

        // Instr 13 (addr 0x34): addi x10, x10, 16
        {instr_mem[55], instr_mem[54], instr_mem[53], instr_mem[52]} = 32'b000000010000_01010_000_01010_0010011;

        // Instr 14 (addr 0x38): addi x11, x11, 16
        {instr_mem[59], instr_mem[58], instr_mem[57], instr_mem[56]} = 32'b000000010000_01011_000_01011_0010011;

        // Instr 15 (addr 0x3C): addi x15, x15, 1      # k++
        {instr_mem[63], instr_mem[62], instr_mem[61], instr_mem[60]} = 32'b000000000001_01111_000_01111_0010011;

        // Instr 16 (addr 0x40): bne x15, x14, -36     # if k != N, goto k_loop (0x1C)
        {instr_mem[67], instr_mem[66], instr_mem[65], instr_mem[64]} = 32'b1111110_01110_01111_001_11001_1100011;

        // Instr 17 (addr 0x44): vse32.v v3, (x12)
        {instr_mem[71], instr_mem[70], instr_mem[69], instr_mem[68]} = 32'b000000100000_01100_110_00011_0100111;

        // Instr 18 (addr 0x48): addi x12, x12, 16
        {instr_mem[75], instr_mem[74], instr_mem[73], instr_mem[72]} = 32'b000000010000_01100_000_01100_0010011;

        // Instr 19 (addr 0x4C): addi x11, x16, 0      # a1 = B_BASE reset
        {instr_mem[79], instr_mem[78], instr_mem[77], instr_mem[76]} = 32'b000000000000_10000_000_01011_0010011;

        // Instr 20 (addr 0x50): addi x13, x13, 1      # i++
        {instr_mem[83], instr_mem[82], instr_mem[81], instr_mem[80]} = 32'b000000000001_01101_000_01101_0010011;

        // Instr 21 (addr 0x54): bne x13, x14, -60     # if i != N, goto row_loop (0x18)
        {instr_mem[87], instr_mem[86], instr_mem[85], instr_mem[84]} = 32'b1111100_01110_01101_001_00101_1100011;

        // Instr 22 (addr 0x58): ebreak
        {instr_mem[91], instr_mem[90], instr_mem[89], instr_mem[88]} = 32'b000000000001_00000_000_00000_1110011;

        instr = {instr_mem[read_addr+3], instr_mem[read_addr+2], instr_mem[read_addr+1], instr_mem[read_addr]};
    end
    else begin
        instr = {instr_mem[read_addr+3], instr_mem[read_addr+2], instr_mem[read_addr+1], instr_mem[read_addr]};
    end
end

endmodule

`timescale 1ns / 1ps

module IF_ID_Reg(
    input clk, input stall, input flush,
    input [31:0] pc_out, input [31:0] instr,
    output reg [31:0] if_id_pc_out, output reg [31:0] if_id_instr,
    output reg [6:0] if_id_opcode, output reg [4:0] if_id_rs1,
    output reg [4:0] if_id_rs2, output reg [4:0] if_id_rd,
    output reg [6:0] if_id_f7, output reg [2:0] if_id_f3
);
always @(posedge clk) begin
    if (stall) begin
        if_id_pc_out <= if_id_pc_out; if_id_instr <= if_id_instr;
        if_id_opcode <= if_id_opcode; if_id_rs1 <= if_id_rs1;
        if_id_rs2 <= if_id_rs2; if_id_rd <= if_id_rd;
        if_id_f7 <= if_id_f7; if_id_f3 <= if_id_f3;
    end
    else if (flush) begin
        if_id_pc_out <= 0; if_id_instr <= 0; if_id_opcode <= 0;
        if_id_rs1 <= 0; if_id_rs2 <= 0; if_id_rd <= 0;
        if_id_f7 <= 0; if_id_f3 <= 0;
    end
    else begin
        if_id_pc_out <= pc_out; if_id_instr <= instr;
        if_id_opcode <= instr[6:0]; if_id_rs1 <= instr[19:15];
        if_id_rs2 <= instr[24:20]; if_id_rd <= instr[11:7];
        if_id_f7 <= instr[31:25]; if_id_f3 <= instr[14:12];
    end
end
endmodule

`timescale 1ns / 1ps

module reg_file(
    input rst, input clk,
    input [4:0] read_reg1, input [4:0] read_reg2,
    input [4:0] write_reg, input [31:0] write_data, input reg_write,
    output reg [31:0] data1, output reg [31:0] data2
);
    reg [31:0] reg_mem [31:0];
    integer i;
    always @(rst) begin
        if (rst) for (i = 0; i < 32; i = i + 1) reg_mem[i] = 0;
    end
       always @(*) begin
        data1 = reg_mem[read_reg1];
        data2 = reg_mem[read_reg2];
    end
    always @(reg_write, write_data, write_reg) begin
        if (write_reg != 0 && reg_write == 1) reg_mem[write_reg] <= write_data;
    end
endmodule

`timescale 1ns / 1ps

module imm_gen(input [31:0] instr, input [1:0] imm_sel, output reg [31:0] imm_val);
reg [11:0] imm_isb;
always@(instr, imm_sel) begin
    if (imm_sel == 2'b00) begin
        imm_isb = instr[31:20];
        imm_val = { {20{imm_isb[11]}}, imm_isb };
    end
    else if (imm_sel == 2'b01) begin
        imm_isb = { instr[31:25], instr[11:7] };
        imm_val = { {20{imm_isb[11]}}, imm_isb };
    end
    else if (imm_sel == 2'b10) begin
        imm_isb = {instr[31], instr[7], instr[30:25], instr[11:8]};
        imm_val = { {20{imm_isb[11]}}, imm_isb };
        imm_val = imm_val << 1;
    end
    else if (imm_sel == 2'b11) begin
        imm_isb = {instr[31], instr[19:12], instr[20], instr[30:21]};
        imm_val = { {20{imm_isb[11]}}, imm_isb };
        imm_val = imm_val << 1;
    end
end
endmodule

`timescale 1ns / 1ps

module ID_EX_Reg(
    input clk, input stall, input flush,
    input [31:0] if_id_pc_out, input [4:0] if_id_rs1, input [4:0] if_id_rs2,
    input [4:0] if_id_rd, input [6:0] if_id_f7, input [2:0] if_id_f3,
    input [31:0] if_id_reg_data1, input [31:0] if_id_reg_data2, input [31:0] if_id_imm_val,
    input if_id_reg_write, input if_id_alu_src, input [1:0] if_id_alu_op,
    input if_id_mem_read, input if_id_mem_write, input if_id_mem_to_reg,
    input if_id_branch, input if_id_jump, input if_id_jalr,
    output reg [31:0] id_ex_reg_data1, output reg [31:0] id_ex_reg_data2,
    output reg [31:0] id_ex_imm_val, output reg [31:0] id_ex_pc_out,
    output reg [4:0] id_ex_rs1, output reg [4:0] id_ex_rs2, output reg [4:0] id_ex_rd,
    output reg [6:0] id_ex_f7, output reg [2:0] id_ex_f3,
    output reg id_ex_reg_write, output reg id_ex_alu_src, output reg [1:0] id_ex_alu_op,
    output reg id_ex_mem_read, output reg id_ex_mem_write, output reg id_ex_mem_to_reg,
    output reg id_ex_branch, output reg id_ex_jump, output reg id_ex_jalr
);
always @(posedge clk) begin
    if (flush) begin
        id_ex_pc_out<=0; id_ex_rs1<=0; id_ex_rs2<=0; id_ex_rd<=0;
        id_ex_f7<=0; id_ex_f3<=0; id_ex_reg_data1<=0; id_ex_reg_data2<=0;
        id_ex_imm_val<=0; id_ex_reg_write<=0; id_ex_alu_src<=0; id_ex_alu_op<=0;
        id_ex_mem_read<=0; id_ex_mem_write<=0; id_ex_mem_to_reg<=0;
        id_ex_branch<=0; id_ex_jump<=0; id_ex_jalr<=0;
    end else  if (stall) begin
        id_ex_pc_out<=if_id_pc_out; id_ex_rs1<=if_id_rs1; id_ex_rs2<=if_id_rs2;
        id_ex_rd<=if_id_rd; id_ex_f7<=if_id_f7; id_ex_f3<=if_id_f3;
        id_ex_reg_data1<=if_id_reg_data1; id_ex_reg_data2<=if_id_reg_data2;
        id_ex_imm_val<=if_id_imm_val; id_ex_reg_write<=if_id_reg_write;
        id_ex_alu_src<=if_id_alu_src; id_ex_alu_op<=if_id_alu_op;
        id_ex_mem_read<=if_id_mem_read; id_ex_mem_write<=if_id_mem_write;
        id_ex_mem_to_reg<=if_id_mem_to_reg; id_ex_branch<=if_id_branch;
        id_ex_jump<=if_id_jump; id_ex_jalr<=if_id_jalr;
    end
end
endmodule

`timescale 1ns / 1ps

module mux_41(input [31:0] in0, [31:0] in1, [31:0] in2, [31:0] in3, [1:0] sel, output reg [31:0] out);
always@(in0, in1, in2, in3, sel)
begin
    case(sel)
        0: out = in0; 1: out = in1; 2: out = in2; 3: out = in3;
        default: out = in0;
    endcase
end
endmodule

`timescale 1ns / 1ps

module alu_ctrl_unit(input [1:0] alu_op, input [2:0] f3, input [6:0] f7, output reg [3:0] alu_ctrl);
always@(alu_op, f3, f7) begin
    if (alu_op == 2'b00) alu_ctrl = 4'b0010;
    else if (alu_op == 2'b01) begin
        if (f3 == 3'b101) begin
            if (f7 == 7'b0000000) alu_ctrl = 4'b0100;
            else if (f7 == 7'b0100000) alu_ctrl = 4'b0101;
        end
        else if (f3 == 3'b000) alu_ctrl = 4'b0010;
        else if (f3 == 3'b110) alu_ctrl = 4'b0001;
        else if (f3 == 3'b111) alu_ctrl = 4'b0000;
        else if (f3 == 3'b001) alu_ctrl = 4'b0011;
    end
    else if (alu_op == 2'b10) begin
        if (f7 == 7'b0000000) begin
            if (f3 == 3'b000) alu_ctrl = 4'b0010;
            else if (f3 == 3'b111) alu_ctrl = 4'b0000;
            else if (f3 == 3'b110) alu_ctrl = 4'b0001;
            else if (f3 == 3'b001) alu_ctrl = 4'b0011;
            else if (f3 == 3'b101) alu_ctrl = 4'b0100;
        end
        else if (f7 == 7'b0100000) begin
            if (f3 == 3'b000) alu_ctrl = 4'b0110;
            else if (f3 == 3'b101) alu_ctrl = 4'b0101;
        end
    end
    else if (alu_op == 2'b11) alu_ctrl = 4'b0110;
end
endmodule

`timescale 1ns / 1ps

module alu(input signed [31:0] a, input signed [31:0] b, input [3:0] alu_ctrl,
           output reg [31:0] alu_result, output zero, neg);
always@(a, b, alu_ctrl) begin
    case(alu_ctrl)
        4'b0000: alu_result = a & b;
        4'b0001: alu_result = a | b;
        4'b0010: alu_result = a + b;
        4'b0011: alu_result = a << b[4:0];
        4'b0100: alu_result = a >> b[4:0];
        4'b0101: alu_result = a >>> b[4:0];
        4'b0110: alu_result = a - b;
    endcase
end
assign zero = (alu_result == 0) ? 1 : 0;
assign neg = alu_result[31];
endmodule

module EX_MEM_Reg(
    input clk,
    input [31:0] id_ex_pc_out, input [4:0] id_ex_rd,
    input [31:0] id_ex_reg_data2, input [31:0] id_ex_alu_result,
    input id_ex_reg_write, input id_ex_mem_read, input id_ex_mem_write,
    input id_ex_mem_to_reg, input id_ex_jump,
    output reg [31:0] ex_mem_pc_out, output reg [4:0] ex_mem_rd,
    output reg [31:0] ex_mem_reg_data2, output reg [31:0] ex_mem_alu_result,
    output reg ex_mem_reg_write, output reg ex_mem_mem_read,
    output reg ex_mem_mem_write, output reg ex_mem_mem_to_reg, output reg ex_mem_jump
);
always @(posedge clk) begin
    ex_mem_pc_out<=id_ex_pc_out; ex_mem_rd<=id_ex_rd;
    ex_mem_reg_data2<=id_ex_reg_data2; ex_mem_alu_result<=id_ex_alu_result;
    ex_mem_reg_write<=id_ex_reg_write; ex_mem_mem_read<=id_ex_mem_read;
    ex_mem_mem_write<=id_ex_mem_write; ex_mem_mem_to_reg<=id_ex_mem_to_reg;
    ex_mem_jump<=id_ex_jump;
end
endmodule

`timescale 1ns / 1ps

module data_mem(input rst, input clk, input [31:0] addr, input [31:0] write_data_dm,
                input mem_read, input mem_write, output reg [31:0] read_data);
reg [31:0] data_mem [99:0];
always@(rst, addr) begin
    if (rst) data_mem[0] = 1;
    else if (mem_read == 1) read_data = data_mem[addr];
end
always@(posedge clk) begin
    if (mem_write == 1) data_mem[addr] = write_data_dm;
end
endmodule

`timescale 1ns / 1ps

module MEM_WB_Reg(
    input clk,
    input [31:0] ex_mem_pc_out, input [4:0] ex_mem_rd,
    input [31:0] ex_mem_read_data, input [31:0] ex_mem_alu_result,
    input ex_mem_reg_write, input ex_mem_mem_to_reg, input ex_mem_jump,
    output reg [31:0] mem_wb_pc_out, output reg [4:0] mem_wb_rd,
    output reg [31:0] mem_wb_read_data, output reg [31:0] mem_wb_alu_result,
    output reg mem_wb_reg_write, output reg mem_wb_mem_to_reg, output reg mem_wb_jump
);
always @(posedge clk) begin
    mem_wb_pc_out<=ex_mem_pc_out; mem_wb_rd<=ex_mem_rd;
    mem_wb_read_data<=ex_mem_read_data; mem_wb_alu_result<=ex_mem_alu_result;
    mem_wb_reg_write<=ex_mem_reg_write; mem_wb_mem_to_reg<=ex_mem_mem_to_reg;
    mem_wb_jump<=ex_mem_jump;
end
endmodule