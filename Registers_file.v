module Registers_file(
        input clk,
        input rst,
        input [4:0] src1,
        input [4:0] src2,
        input [4:0] dest,
        input [31:0] Write_Val,
        input Write_En,

        output [31:0] reg1,
        output [31:0] reg2
    );
    integer i;
    reg [31:0] registers [31:0];
    assign reg1 = registers[src1];
    assign reg2 = registers[src2];

    always@(negedge clk, posedge rst)begin
          if(rst)begin
               for(i = 0; i < 32; i = i+1)begin
                    registers[i] <= i;
               end
          end
          else if(Write_En && dest != 0)begin
               registers[dest] <= Write_Val;
          end
     end

endmodule //Registers_file
