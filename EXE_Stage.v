module EXE_Stage(
        input clk,
        input [3:0] EXE_CMD,
        input [31:0] val1,
        input [31:0] val2,
        input [31:0] val_src2,
        input [31:0] PC,
        input [1:0] Br_type,
        input [1:0] val1_sel,
        input [1:0] val2_sel,
        input [1:0] src2_sel,
        input [31:0] ALU_result_in,
        input [31:0] WB_result_in,

        output [31:0] ALU_result,
        output [31:0] Br_addr,
        output [31:0] val_src2_out,
        output Br_taken
    );

    wire [31:0] val1_Mux_out, val2_Mux_out;

    Mux5 val1_Mux(.sel(val1_sel), .in0(val1), .in1(ALU_result_in), .in2(WB_result_in), .out(val1_Mux_out));
    Mux5 val2_Mux(.sel(val2_sel), .in0(val2), .in1(ALU_result_in), .in2(WB_result_in), .out(val2_Mux_out));
    Mux5 src2_Mux(.sel(src2_sel), .in0(val_src2), .in1(ALU_result_in), .in2(WB_result_in), .out(val_src2_out));

    ALU ALU(.val1(val1_Mux_out), .val2(val2_Mux_out), .EXE_CMD(EXE_CMD), .ALU_result(ALU_result));

    condition_check condition_check(.branch_type(Br_type), .val1(val1_Mux_out), .src2_val(val_src2_out), .branch_taken(Br_taken));

    Adder PC_adder(.in1(val2_Mux_out << 2), .in2(PC), .out(Br_addr));

endmodule //EXE_Stage
