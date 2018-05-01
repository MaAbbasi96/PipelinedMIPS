module ID_Stage (
          input clk,
          input rst,
          input hazard_detection,
          input [31:0] Instruction,
          input [31:0] reg1,
          input [31:0] reg2,
          output [4:0] src1,
          output [4:0] src2,
          output [4:0] src2_reg_file,
          output [4:0] Dest,
          output [31:0] Reg2,
          output [31:0] Val2,
          output [31:0] Val1,
          output [1:0] Br_type,
          output [3:0] EXE_CMD,
          output MEM_R_EN,
          output MEM_W_EN,
          output WB_EN,
          output one_input
     );

     wire CU_is_imm;
     wire[4:0] three_wire, zero_wire, src2_in;
     wire[31:0] SE_immediate_ext;
     wire[1:0] CU_Br_type;
     wire CU_MEM_R_EN, CU_MEM_W_EN, CU_WB_EN;

     assign Reg2 = reg2;

     assign three_wire = 5'd3;
     assign zero_wire = 5'd0;

     assign Val1 = reg1;

     assign src1 = Instruction[25:21];
     assign src2_in = Instruction[20:16];

     ControlUnit CU(.opcode(Instruction[31:26]), .EXE_CMD(EXE_CMD), .br_type(CU_Br_type),
                    .mem_read(CU_MEM_R_EN), .mem_write(CU_MEM_W_EN), .wb_en(CU_WB_EN), .is_imm(CU_is_imm), .one_input(one_input));

     SignExt SE(.immediate(Instruction[15:0]), .immediate_ext(SE_immediate_ext));

     Mux2 Val_Mux(.sel(CU_is_imm), .in0(reg2), .in1(SE_immediate_ext), .out(Val2));

     Mux2_5bit Dest_Mux(.sel(CU_is_imm), .in0(Instruction[15:11]), .in1(Instruction[20:16]), .out(Dest));

     Mux2_5bit src2_Mux(.sel(CU_is_imm), .in0(src2_in), .in1(zero_wire), .out(src2));

     assign src2_reg_file = src2_in;

     Mux2_5bit CU_Mux(.sel(hazard_detection), .in0({CU_MEM_R_EN, CU_MEM_W_EN, CU_WB_EN, CU_Br_type}), .in1(three_wire), .out({MEM_R_EN, MEM_W_EN, WB_EN, Br_type}));

endmodule // ID_Stage
