module ALU (
        input [31:0] val1,
        input [31:0] val2,
        input [3:0] EXE_CMD,
        output [31:0] ALU_result
    );

    parameter ADD = 4'd0, SUB = 4'd2, AND = 4'd4, OR = 4'd5, NOR = 4'd6,
    XOR = 4'd7, SHIFT_LEFT = 4'd8, SHIFT_ART = 4'd9, SHIFT_LOG = 4'd10;

    assign ALU_result = (EXE_CMD == ADD)? val1 + val2:
                    (EXE_CMD == SUB)? val1 - val2:
                    (EXE_CMD == AND)? val1 & val2:
                    (EXE_CMD == OR)? val1 | val2:
                    (EXE_CMD == NOR)? ~(val1 | val2):
                    (EXE_CMD == XOR)? val1 ^ val2:
                    (EXE_CMD == SHIFT_LEFT)? val1 << val2:
                    (EXE_CMD == SHIFT_ART)? val1 >>> val2:
                    (EXE_CMD == SHIFT_LOG)? val1 >> val2: 0;

endmodule // ALU
