module MEM_Stage (
          input clk,
          input rst,
          input MEM_R_EN,
          input MEM_W_EN,
          input [31:0] ALU_result_in,
          input [31:0] ST_val,
          output [31:0] Mem_read_value,
          output ready,
          inout [15:0]	SRAM_DQ,				//	SRAM Data bus 16 Bits
          output [17:0]	SRAM_ADDR,				//	SRAM Address bus 18 Bits
          output SRAM_UB_N,				//	SRAM High-byte Data Mask
          output SRAM_LB_N,				//	SRAM Low-byte Data Mask
          output SRAM_WE_N,				 //SRAM Write Enable
          output SRAM_CE_N,				 //SRAM Chip Enable
          output SRAM_OE_N
     );

     // Data_memory dm(.clk(clk), .MEM_W_EN(MEM_W_EN), .MEM_R_EN(MEM_R_EN), .ALU_result(ALU_result_in), .ST_value(ST_val), .MEM_OUT(Mem_read_value));
     SRAM_Controller SC(.clk(clk), .rst(rst), .wr_en(MEM_W_EN), .rd_en(MEM_R_EN), .address(ALU_result_in), .writeData(ST_val), .readData(Mem_read_value), .ready(ready),
                    .SRAM_DQ(SRAM_DQ), .SRAM_ADDR(SRAM_ADDR), .SRAM_UB_N(SRAM_UB_N), .SRAM_LB_N(SRAM_LB_N), .SRAM_WE_N(SRAM_WE_N), .SRAM_CE_N(SRAM_CE_N), .SRAM_OE_N(SRAM_OE_N));

endmodule // MEM_Stage
