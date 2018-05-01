module ID_Stage_reg (
          input clk,
          input rst,
          input flush,
          input [4:0] Dest_in,
          input [31:0] Reg2_in,
          input [31:0] Val2_in,
          input [31:0] Val1_in,
          input [31:0] PC_in,
          input [1:0] Br_type_in,
          input [3:0] EXE_CMD_in,
          input MEM_R_EN_in,
          input MEM_W_EN_in,
          input WB_EN_in,
          input [4:0] src1_in,
          input [4:0] src2_in,
          input freeze,

          output reg [4:0] Dest,
          output reg [31:0] Reg2,
          output reg [31:0] Val2,
          output reg [31:0] Val1,
          output reg [31:0] PC_out,
          output reg [1:0] Br_type,
          output reg [3:0] EXE_CMD,
          output reg MEM_R_EN,
          output reg MEM_W_EN,
          output reg WB_EN,
          output reg [4:0] src1,
          output reg [4:0] src2
     );

     always @(posedge clk) begin
        if(rst | flush) begin
            Dest <= 5'b0;
            Reg2 <= 32'b0;
            Val2 <= 32'b0;
            Val1 <= 32'b0;
            PC_out <= 32'b0;
            Br_type <= 2'b11;
            EXE_CMD <= 4'b0;
            MEM_R_EN <= 1'b0;
            src1 <= 5'b0;
            MEM_W_EN <= 1'b0;
            src2 <= 5'b0;
            WB_EN <= 1'b0;
        end
        else if(!freeze) begin
            Dest <= Dest_in;
            Reg2 <= Reg2_in;
            Val2 <= Val2_in;
            Val1 <= Val1_in;
            PC_out <= PC_in;
            Br_type <= Br_type_in;
            EXE_CMD <= EXE_CMD_in;
            MEM_R_EN <= MEM_R_EN_in;
            src1 <= src1_in;
            MEM_W_EN <= MEM_W_EN_in;
            src2 <= src2_in;
            WB_EN <= WB_EN_in;
        end
     end

endmodule // ID_Stage_reg
