module ControlUnit (
        input [5:0] opcode,
        output[3:0] EXE_CMD,
        output[1:0] br_type,
        output mem_read,
        output mem_write,
        output wb_en,
        output is_imm,
        output one_input
    );

    parameter NOP = 6'd0, ADD = 6'd1, SUB = 6'd3, AND = 6'd5, OR = 6'd6, NOR = 6'd7,
    XOR = 6'd8, SLA = 6'd9, SLL = 6'd10, SRA = 6'd11, SRL = 6'd12, ADDI = 6'd32,
    SUBI = 6'd33, LD = 6'd36, ST = 6'd37, BEZ = 6'd40, BNE = 6'd41, JMP = 6'd42;

    assign EXE_CMD = (opcode == NOP)? 4'bxxxx:
                (opcode == ADD)? 0:
				(opcode == SUB)? 2:
				(opcode == AND)? 4:
				(opcode == OR)? 5:
				(opcode == NOR)? 6:
				(opcode == XOR)? 7:
				(opcode == SLA)? 8:
				(opcode == SLL)? 8:
				(opcode == SRA)? 9:
				(opcode == SRL)? 10:
				(opcode == ADDI)? 0:
				(opcode == SUBI)? 2:
				(opcode == LD)? 0:
				(opcode == ST)? 0:
				(opcode == BEZ)? 4'bxxxx:
				(opcode == BNE)? 4'bxxxx:
				(opcode == JMP)?0: 4'bxxxx;

    assign is_imm = (opcode == ADDI)? 1:
                (opcode == SUBI)? 1:
                (opcode == LD)? 1:
                (opcode == ST)? 1:
                (opcode == BEZ)? 1:
                (opcode == BNE)? 1:
                (opcode == JMP)? 1: 0;

    assign one_input = (opcode == ADDI)? 1:
                (opcode == SUBI)? 1:
                (opcode == LD)? 1:
                (opcode == BEZ)? 1:
                (opcode == JMP)? 1: 0;

    assign mem_read = (opcode == LD)? 1: 0;

    assign mem_write = (opcode == ST)? 1: 0;

    assign wb_en = (opcode == NOP)? 0:
                    (opcode == ST)? 0:
                    (opcode == BEZ)? 0:
                    (opcode == BNE)? 0:
                    (opcode == JMP)? 0: 1;

    assign br_type=(opcode == BEZ)? 0:
                        (opcode == BNE)? 1:
                        (opcode == JMP)? 2: 3;

endmodule
