module Mux5 (input[1:0] sel, input [31:0] in0, input [31:0] in1, input [31:0] in2, output [31:0] out);
     assign out = (sel == 2'd0) ? in0 :
                (sel == 2'd1) ? in1 : in2;
endmodule // Mux5
