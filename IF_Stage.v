module IF_Stage (
          input clk,
          input rst,
          input Br_taken,
          input freeze,
          input [31:0] Br_Addr,
          output [31:0] PC,
          output [31:0] Instruction
     );

     wire[31:0] New_PC, reg_out, four_wire;

     assign four_wire = 32'd4;

     Register PC_register(.clk(clk), .rst(rst), .in(New_PC), .freeze(freeze), .out(reg_out));
     Mux2 PC_mux(.sel(Br_taken), .in0(PC), .in1(Br_Addr), .out(New_PC));
     Adder PC_adder(.in1(reg_out), .in2(four_wire), .out(PC));
     Memory Instruction_memory(.clk(clk), .rst(rst), .PC(reg_out), .instruction(Instruction));

endmodule // IF_Stage
