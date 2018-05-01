module Register (
          input clk,
          input rst,
          input freeze,
          input [31:0] in,
          output reg [31:0] out
     );

     always @ ( posedge clk ) begin
          if(rst)
               out <= 0;
          else if(!freeze)
               out <= in;
     end

endmodule // Register
