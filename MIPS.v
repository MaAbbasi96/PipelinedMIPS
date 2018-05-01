// ============================================================================
// Copyright (c) 2012 by Terasic Technologies Inc.
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development
//   Kits made by Terasic.  Other use of this code, including the selling
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use
//   or functionality of this code.
//
// ============================================================================
//
//  Terasic Technologies Inc
//  9F., No.176, Sec.2, Gongdao 5th Rd, East Dist, Hsinchu City, 30070. Taiwan
//
//
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// ============================================================================
//
// Major Functions:	DE2 TOP LEVEL
//
// ============================================================================
//
// Revision History :
// ============================================================================
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Johnny Chen       :| 05/08/19  :|      Initial Revision
//   V1.1 :| Johnny Chen       :| 05/11/16  :|      Added FLASH Address FL_ADDR[21:20]
//   V1.2 :| Johnny Chen       :| 05/11/16  :|		Fixed ISP1362 INT/DREQ Pin Direction.
//   V1.3 :| Johnny Chen       :| 06/11/16  :|		Added the Dedicated TV Decoder Line-Locked-Clock Input
//													            for DE2 v2.X PCB.
//   V1.5 :| Eko    Yan        :| 12/01/30  :|      Update to version 11.1 sp1.
// ============================================================================

module MIPS
	(
		////////////////////	Clock Input	 	////////////////////
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		//UART_TXD,						//	UART Transmitter
		//UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		//IRDA_TXD,						//	IRDA Transmitter
		//IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask
		SRAM_LB_N,						//	SRAM Low-byte Data Mask
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		//SD_DAT,							//	SD Card Data
		//SD_WP_N,						   //	SD Write protect
		//SD_CMD,							//	SD Card Command Signal
		//SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	   TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		TD_CLK27,                  //	TV Decoder 27MHz CLK
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input		   	CLOCK_27;				//	27 MHz
input		   	CLOCK_50;				//	50 MHz
input			   EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	   [3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	  [17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]	LEDG;					//	LED Green[8:0]
output  [17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
//output			UART_TXD;				//	UART Transmitter
//input			   UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
//output			IRDA_TXD;				//	IRDA Transmitter
//input			   IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	  [15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output  [11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	  [7:0]	FL_DQ;					//	FLASH Data bus 8 Bits
output [21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	 [15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output [17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	 [15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output  [1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input			   OTG_INT0;				//	ISP1362 Interrupt 0
input			   OTG_INT1;				//	ISP1362 Interrupt 1
input			   OTG_DREQ0;				//	ISP1362 DMA Request 0
input			   OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	  [7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
//inout	 [3:0]	SD_DAT;					//	SD Card Data
//input			   SD_WP_N;				   //	SD write protect
//inout			   SD_CMD;					//	SD Card Command Signal
//output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			   I2C_SDAT;				//	I2C Data
output			I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	   PS2_DAT;				//	PS2 Data
input			   PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input			   ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			   AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			   AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			   AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output			AUD_DACDAT;				//	Audio CODEC DAC Data
inout			   AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output			AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	 [7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			   TD_HS;					//	TV Decoder H_SYNC
input			   TD_VS;					//	TV Decoder V_SYNC
output			TD_RESET;				//	TV Decoder Reset
input          TD_CLK27;            //	TV Decoder 27MHz CLK
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1

     wire[31:0] zero_wire_32;
		 wire[31:0] IF_PC, IF_Instruction;
		 wire[31:0] IF_reg_PC, IF_reg_Instruction;
		 wire[31:0] ID_Reg2, ID_Val2, ID_Val1;
		 wire[31:0] ID_reg_Reg2, ID_reg_Val2, ID_reg_Val1, ID_reg_PC;
     wire[31:0] RF_val1, RF_val2;
		 wire[31:0] EXE_ALU_result, EXE_Br_addr;
		 wire[31:0] EXE_reg_PC, EXE_reg_ALU_result, EXE_reg_ST_val, EXE_src2;
		 wire[31:0] MEM_Mem_read_value;
		 wire[31:0] MEM_reg_ALU_result, MEM_reg_MEM_read_value;
		 wire[31:0] WB_Write_value;
     wire[4:0] ID_src1, ID_src2, ID_Dest, ID_EXE_CMD, ID_src2_reg_file;
		 wire[4:0] ID_reg_Dest, ID_reg_EXE_CMD, ID_reg_src1, ID_reg_src2;
		 wire[4:0] EXE_reg_Dest;
		 wire[4:0] MEM_reg_Dest;
		 wire[4:0] WB_Dest;
     wire[1:0] ID_Br_type;
		 wire[1:0] ID_reg_Br_type;
		 wire[1:0] fwd_to_val1, fwd_to_val2, fwd_to_src2;
     wire zero_wire_1;
		 wire rst;
		 wire ID_MEM_R_EN, ID_MEM_W_EN, ID_WB_EN, ID_one_input;
		 wire ID_reg_MEM_R_EN, ID_reg_MEM_W_EN, ID_reg_WB_EN;
		 wire EXE_Br_taken;
		 wire EXE_reg_WB_en, EXE_reg_MEM_R_EN, EXE_reg_MEM_W_EN;
		 wire MEM_ready;
		 wire MEM_reg_WB_en, MEM_reg_MEM_R_EN;
		 wire WB_WB_en;
	wire hazard_detection;

     assign rst = SW[0];
     assign zero_wire_1 = 1'b0;
     assign zero_wire_32 = 32'b0;


     IF_Stage IF_Stage(.clk(CLOCK_50), .rst(rst), .Br_taken(EXE_Br_taken), .Br_Addr(EXE_Br_addr), .freeze(hazard_detection | (!MEM_ready)), .PC(IF_PC), .Instruction(IF_Instruction));

     IF_Stage_reg IF_Stage_reg(.clk(CLOCK_50), .rst(rst), .flush(EXE_Br_taken), .PC_in(IF_PC),
                                   .Instruction_in(IF_Instruction), .freeze(hazard_detection | (!MEM_ready)), .PC(IF_reg_PC), .Instruction(IF_reg_Instruction));

     ID_Stage ID_Stage(.clk(CLOCK_50), .rst(rst), .hazard_detection(hazard_detection), .Instruction(IF_reg_Instruction), .reg1(RF_val1), .reg2(RF_val2),
                         .src1(ID_src1), .src2(ID_src2), .Dest(ID_Dest), .Reg2(ID_Reg2), .Val2(ID_Val2), .Val1(ID_Val1),
                         .Br_type(ID_Br_type), .EXE_CMD(ID_EXE_CMD), .MEM_R_EN(ID_MEM_R_EN), .MEM_W_EN(ID_MEM_W_EN), .WB_EN(ID_WB_EN), .one_input(ID_one_input),
						 .src2_reg_file(ID_src2_reg_file));

     ID_Stage_reg ID_Stage_reg(.clk(CLOCK_50), .rst(rst), .flush(EXE_Br_taken), .Dest_in(ID_Dest), .Reg2_in(ID_Reg2), .Val2_in(ID_Val2), .Val1_in(ID_Val1),
                                   .PC_in(IF_reg_PC), .Br_type_in(ID_Br_type), .EXE_CMD_in(ID_EXE_CMD), .MEM_R_EN_in(ID_MEM_R_EN), .MEM_W_EN_in(ID_MEM_W_EN),
                                   .WB_EN_in(ID_WB_EN), .Dest(ID_reg_Dest), .Reg2(ID_reg_Reg2), .Val2(ID_reg_Val2), .Val1(ID_reg_Val1),
                                   .PC_out(ID_reg_PC), .Br_type(ID_reg_Br_type), .EXE_CMD(ID_reg_EXE_CMD), .MEM_R_EN(ID_reg_MEM_R_EN),
                                   .MEM_W_EN(ID_reg_MEM_W_EN), .WB_EN(ID_reg_WB_EN),
								   .src1(ID_reg_src1), .src2(ID_reg_src2), .src1_in(ID_src1), .src2_in(ID_src2),
								   .freeze(!MEM_ready));

     Registers_file Registers_file(.clk(CLOCK_50), .rst(rst), .src1(ID_src1), .src2(ID_src2_reg_file), .dest(WB_Dest), .Write_Val(WB_Write_value), .Write_En(WB_WB_en),
		 																.reg1(RF_val1), .reg2(RF_val2));

	 EXE_Stage EXE_Stage(.clk(CLOCK_50), .EXE_CMD(ID_reg_EXE_CMD), .val1(ID_reg_Val1), .val2(ID_reg_Val2), .val_src2(ID_reg_Reg2),
		 														.PC(ID_reg_PC), .Br_type(ID_reg_Br_type), .ALU_result(EXE_ALU_result), .Br_addr(EXE_Br_addr), .Br_taken(EXE_Br_taken),
																.val1_sel(fwd_to_val1), .val2_sel(fwd_to_val2), .src2_sel(fwd_to_src2),
																.ALU_result_in(EXE_reg_ALU_result), .WB_result_in(WB_Write_value), .val_src2_out(EXE_src2));

	EXE_Stage_reg EXE_Stage_reg(.clk(CLOCK_50), .rst(rst), .WB_en_in(ID_reg_WB_EN), .MEM_R_EN_in(ID_reg_MEM_R_EN), .MEM_W_EN_in(ID_reg_MEM_W_EN), .PC_in(ID_reg_PC),
		  															.ALU_result_in(EXE_ALU_result), .ST_val_in(EXE_src2), .Dest_in(ID_reg_Dest), .WB_en(EXE_reg_WB_en), .MEM_R_EN(EXE_reg_MEM_R_EN),
																		.MEM_W_EN(EXE_reg_MEM_W_EN), .PC(EXE_reg_PC), .ALU_result(EXE_reg_ALU_result), .ST_val(EXE_reg_ST_val), .Dest(EXE_reg_Dest),
																		.freeze(!MEM_ready));

	MEM_Stage MEM_Stage(.clk(CLOCK_50), .MEM_R_EN(EXE_reg_MEM_R_EN), .MEM_W_EN(EXE_reg_MEM_W_EN), .ALU_result_in(EXE_reg_ALU_result), .ST_val(EXE_reg_ST_val), .Mem_read_value(MEM_Mem_read_value),
							.SRAM_DQ(SRAM_DQ), .SRAM_ADDR(SRAM_ADDR), .SRAM_UB_N(SRAM_UB_N), .SRAM_LB_N(SRAM_LB_N), .SRAM_WE_N(SRAM_WE_N), .SRAM_CE_N(SRAM_CE_N), .SRAM_OE_N(SRAM_OE_N), .ready(MEM_ready),
							.rst(rst));

	MEM_Stage_reg MEM_Stage_reg(.clk(CLOCK_50), .rst(rst), .WB_en_in(EXE_reg_WB_en), .MEM_R_EN_in(EXE_reg_MEM_R_EN), .ALU_result_in(EXE_reg_ALU_result),
		 															.MEM_read_value_in(MEM_Mem_read_value), .Dest_in(EXE_reg_Dest), .WB_en(MEM_reg_WB_en), .MEM_R_EN(MEM_reg_MEM_R_EN),
																	.ALU_result(MEM_reg_ALU_result), .MEM_read_value(MEM_reg_MEM_read_value), .Dest(MEM_reg_Dest),
																	.freeze(!MEM_ready));

     WB_Stage WB_Stage(.clk(CLOCK_50), .WB_en_in(MEM_reg_WB_en), .MEM_R_EN(MEM_reg_MEM_R_EN), .ALU_result(MEM_reg_ALU_result), .MEM_read_value(MEM_reg_MEM_read_value),
                    .Dest_in(MEM_reg_Dest), .WB_en(WB_WB_en), .Write_value(WB_Write_value), .Dest(WB_Dest));

	hazard_detection_unit hdu(.src1(ID_src1), .src2(ID_src2_reg_file), .exe_dest(ID_reg_Dest), .exe_wb_en(ID_reg_WB_EN), .mem_dest(EXE_reg_Dest), .mem_wb_en(EXE_reg_WB_en),
	 				.one_input(ID_one_input), .hazard_detection(hazard_detection), .forward_enable(SW[3]), .MEM_R_EN(ID_reg_MEM_R_EN));


	forward_unit fwd(.src1(ID_reg_src1), .src2(ID_reg_src2), .dest(ID_reg_Dest), .mem_dest(EXE_reg_Dest), .write_back_dest(WB_Dest), .mem_wb(EXE_reg_WB_en),
	 				.write_back_wb(WB_WB_en), .forward_enable(SW[3]), .to_val1_mux(fwd_to_val1), .to_val2_mux(fwd_to_val2), .to_src2_mux(fwd_to_src2));


endmodule
