module IF_Stage_reg (
          input clk,
          input rst,
          input flush,
          input freeze,
          input [31:0] PC_in,
          input [31:0] Instruction_in,
          output reg [31:0] PC,
          output reg [31:0] Instruction
     );

     always @(posedge clk) begin
        if(rst | flush) begin
            PC <= 32'b0;
            Instruction <= 32'b0;
        end
        else if(!freeze) begin
            PC <= PC_in;
            Instruction <= Instruction_in;
        end
    end

endmodule // IF_Stage_reg
