module Data_memory(
        input clk,
        input MEM_W_EN, MEM_R_EN,
        input [31:0] ALU_result,
        input [31:0] ST_value,

        output [31:0] MEM_OUT);

    integer i;
    reg [31:0] data [63:0];

    assign MEM_OUT = MEM_R_EN ? data[(ALU_result>>2)-256]: 32'd0;

    always@(posedge clk)begin
          if(MEM_W_EN)begin
               data[(ALU_result>>2)-256] = ST_value;
          end
     end

endmodule
