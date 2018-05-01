module WB_Stage(
        input clk,
        input WB_en_in,
        //MEM_Signals
        input MEM_R_EN,
        //memory Address
        input [31:0] ALU_result,

        input [31:0] MEM_read_value,
        input [4:0] Dest_in,

        output WB_en,
        output [31:0] Write_value,
        output [4:0] Dest
    );

    assign Dest = Dest_in;

    assign WB_en = WB_en_in;

    Mux2 WB_Mux(.sel(MEM_R_EN), .in0(ALU_result), .in1(MEM_read_value), .out(Write_value));

endmodule //WB_Stage
