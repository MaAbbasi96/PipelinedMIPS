module Memory (input clk, input rst, input [31:0] PC, output reg [31:0] instruction);
     reg[31:0] instructions[255:0];

     always @ ( posedge clk ) begin
          if(rst) begin
            // instructions[0]=0;
            // instructions[1]=32'b100000_00000_00001_00000_11000001010;//-- Addi r0 ,1546 ,r1
            // instructions[2]=0;
            // instructions[3]=0;
            // instructions[4]=32'b000001_00000_00001_00010_00000000000;//-- Add r0 ,r1 ,r2;
            // instructions[5]=32'b000011_00000_00001_00011_00000000000;//-- sub r0 ,r1 ,r3;
            // instructions[6]=0;
            // instructions[7]=0;
            // instructions[8]=32'b000101_00010_00011_00100_00000000000;//-- And r2 ,r3 ,r4
            // instructions[9]=32'b100001_00011_00101_00000_01000110100;//-- Subi r3 ,r5 ,564
            // instructions[10]=0;
            // instructions[11]=32'b000110_00011_00100_00101_00000000000;//-- or r3 ,r4 ,r5
            // instructions[12]=0;
            // instructions[13]=0;
            // instructions[14]=32'b000111_00101_00000_00110_00000000000;//-- nor r5 ,r0 ,r6
            // instructions[15]=32'b000111_00100_00000_01011_00000000000;//-- nor r4 ,r0 ,r11
            // instructions[16]=32'b000011_00101_00101_00101_00000000000;//-- sub r5 ,r5 ,r5
            // instructions[17]=32'b100000_00000_00001_00000_10000000000;//-- Addi r0 ,r1 ,1024
            // instructions[18]=0;
            // instructions[19]=0;
            // instructions[20]=32'b100101_00001_00010_00000_00000000000;//-- st r1 ,r2 ,0
            // instructions[21]=32'b100100_00001_00101_00000_00000000000;//-- ld r1 ,r5 ,0
            // instructions[22]=0;
            // instructions[23]=0;
            // instructions[24]=32'b101000_01001_00000_00000_00000000001;//-- Bez r9 , 1
            // instructions[25]=32'b001000_00101_00001_00111_00000000000;//-- xor r5 ,r1 ,r7
            // instructions[26]=32'b001000_00101_00001_00000_00000000000;//-- xor r5 ,r1 ,r0
            // instructions[27]=32'b001001_00011_01011_00111_00000000000;//-- sla r3 ,r11 ,r7
            // instructions[28]=32'b001010_00011_01011_01000_00000000000;//-- sll r3 ,r11 ,r8
            // instructions[29]=32'b001011_00011_00100_01001_00000000000;//-- sra r3 ,r4 ,r9
            // instructions[30]=32'b001100_00011_00100_01010_00000000000;//-- srl r3 ,r4 ,r10
            // instructions[31]=32'b100101_00001_00011_00000_00000000100;//-- st r1 ,r3 ,4
            // instructions[32]=32'b100101_00001_00100_00000_00000001000;//-- st r1 ,r4 ,8
            // instructions[33]=32'b100101_00001_00101_00000_00000001100;//-- st r1 ,r5 ,12
            // instructions[34]=32'b100101_00001_00110_00000_00000010000;//-- st r1 ,r6 ,16
            // instructions[35]=32'b100100_00001_01011_00000_00000000100;//-- ld r1 ,r11 ,4
            // instructions[36]=32'b100101_00001_00111_00000_00000010100;//-- st r1 ,r7 ,20
            // instructions[37]=32'b100101_00001_01000_00000_00000011000;//-- st r1 ,r8 ,24
            // instructions[38]=32'b100101_00001_01001_00000_00000011100;//-- st r1 ,r9 ,28
            // instructions[39]=32'b100101_00001_01010_00000_00000100000;//-- st r1 ,r10 ,32
            // instructions[40]=32'b100101_00001_01011_00000_00000100100;//-- st r1 ,r11 ,36
            // instructions[41]=32'b100000_00000_00001_00000_00000000011;//-- Addi r0 ,r1 ,3
            // instructions[42]=32'b100000_00000_00100_00000_10000000000;//-- Addi r0 ,r4 ,1024
            // instructions[43]=32'b100000_00000_00010_00000_00000000000;//-- Addi r0 ,r2 ,0
            // instructions[44]=32'b100000_00000_00011_00000_00000000001;//-- Addi r0 ,r3 ,1 //////////////////
            // instructions[45]=32'b100000_00000_01001_00000_00000000010;//-- Addi r0 ,r9 ,2 ////////++++
            // instructions[46]=0;
            // instructions[47]=0;
            // instructions[48]=32'b001010_00011_01001_01000_00000000000;//-- sll r3 ,r9 ,r8
            // instructions[49]=0;
            // instructions[50]=0;
            // instructions[51]=0;
            // instructions[52]=2'b000001_00100_01000_01000_00000000000;//-- Add r4 ,r8 ,r8
            // instructions[53]=0;
            // instructions[54]=0;
            // instructions[55]=32'b100100_01000_00101_00000_00000000000;//-- ld r8 ,r5 ,0
            // instructions[56]=32'b100100_01000_00110_11111_11111111100;//-- ld r8 ,r6 ,-4
            // instructions[57]=0;
            // instructions[58]=0;
            // instructions[59]=32'b000011_00101_00110_01001_00000000000;//-- sub r5 ,r6 ,r9
            // instructions[60]=32'b100000_00000_01010_10000_00000000000;//-- Addi r0 ,r10 ,0x8000
            // instructions[61]=32'b100000_00000_01011_00000_00000010000;//-- Addi r0 ,r11 ,16
            // instructions[62]=0;
            // instructions[63]=0;
            // instructions[64]=32'b001010_01010_01011_01010_00000000000;//-- sll r10 ,r11 ,r10
            // instructions[65]=0;
            // instructions[66]=0;
            // instructions[67]=32'b000101_01001_01010_01001_00000000000;//-- And r9 ,r10 ,r9
            // instructions[68]=0;
            // instructions[69]=0;
            // instructions[70]=32'b101000_01001_00000_00000_00000000010;//-- Bez r9 ,2
            // instructions[71]=32'b100101_01000_00101_11111_11111111100;//-- st r8 ,r5 ,-4
            // instructions[72]=32'b100101_01000_00110_00000_00000000000;//-- st r8 ,r6 ,0
            // instructions[73]=32'b100000_00011_00011_00000_00000000001;//-- Addi r3 ,r3 ,1
            // instructions[74]=0;
            // instructions[75]=0;
            // instructions[76]=32'b101001_00001_00011_11111_11111100000;//-- BNE r1 ,r3 ,-32 //////// 76 - 45 + 1
            // instructions[77]=0;
            // instructions[78]=0;
            // instructions[79]=32'b100000_00010_00010_00000_00000000001;//-- Addi r2 ,r2 ,1;;
            // instructions[80]=0;
            // instructions[81]=0;
            // instructions[82]=32'b101001_00001_00010_11111_11111011001;//-- BNE r1 ,r2 ,-39  //////////////////// 82 - 44 + 1
            // instructions[83]=0;
            // instructions[84]=0;
            // instructions[85]=32'b100000_00000_00001_00000_10000000000;//-- Addi r0 ,r1 ,1024
            // instructions[86]=0;
            // instructions[87]=0;
            // instructions[88]=32'b100100_00001_00010_00000_00000000000;//-- ld r1 ,r2 ,0
            // instructions[89]=32'b100100_00001_00011_00000_00000000100;//-- ld r1 ,r3 ,4
            // instructions[90]=32'b100100_00001_00100_00000_00000001000;//-- ld r1 ,r4 ,8
            // instructions[91]=32'b100100_00001_00101_00000_00000001100;//-- ld r1 ,r5 ,12
            // instructions[92]=32'b100100_00001_00110_00000_00000010000;//-- ld r1 ,r6 ,16
            // instructions[93]=32'b100100_00001_00111_00000_00000010100;//-- ld r1 ,r7 ,20
            // instructions[94]=32'b100100_00001_01000_00000_00000011000;//-- ld r1 ,r8 ,24
            // instructions[95]=32'b100100_00001_01001_00000_00000011100;//-- ld r1 ,r9 ,28
            // instructions[96]=32'b100100_00001_01010_00000_00000100000;//-- ld r1 ,r10 ,32
            // instructions[97]=32'b100100_00001_01011_00000_00000100100;//-- ld r1 ,r11 ,36
            // instructions[98]=32'b101010_00000_00000_11111_11111111111;//-- JMP -1
            instructions[0]=32'b100000_00000_00001_00000_11000001010;//-- Addi r0 ,1546 ,r1
            instructions[1]=32'b000001_00000_00001_00010_00000000000;//-- Add r0 ,r1 ,r2;
            instructions[2]=32'b000011_00000_00001_00011_00000000000;//-- sub r0 ,r1 ,r3;
            instructions[3]=32'b000101_00010_00011_00100_00000000000;//-- And r2 ,r3 ,r4
            instructions[4]=32'b100001_00011_00101_00000_01000110100;//-- Subi r3 ,r5 ,564
            instructions[5]=32'b000110_00011_00100_00101_00000000000;//-- or r3 ,r4 ,r5
            instructions[6]=32'b000111_00101_00000_00110_00000000000;//-- nor r5 ,r0 ,r6
            instructions[7]=32'b000111_00100_00000_01011_00000000000;//-- nor r4 ,r0 ,r11
            instructions[8]=32'b000011_00101_00101_00101_00000000000;//-- sub r5 ,r5 ,r5
            instructions[9]=32'b100000_00000_00001_00000_10000000000;//-- Addi r0 ,r1 ,1024
            instructions[10]=32'b100101_00001_00010_00000_00000000000;//-- st r1 ,r2 ,0
            instructions[11]=32'b100100_00001_00101_00000_00000000000;//-- ld r1 ,r5 ,0
            instructions[12]=32'b101000_01001_00000_00000_00000000001;//-- Bez r9 , 1
            instructions[13]=32'b001000_00101_00001_00111_00000000000;//-- xor r5 ,r1 ,r7
            instructions[14]=32'b001000_00101_00001_00000_00000000000;//-- xor r5 ,r1 ,r0
            instructions[15]=32'b001001_00011_01011_00111_00000000000;//-- sla r3 ,r11 ,r7
            instructions[16]=32'b001010_00011_01011_01000_00000000000;//-- sll r3 ,r11 ,r8
            instructions[17]=32'b001011_00011_00100_01001_00000000000;//-- sra r3 ,r4 ,r9
            instructions[18]=32'b001100_00011_00100_01010_00000000000;//-- srl r3 ,r4 ,r10
            instructions[19]=32'b100101_00001_00011_00000_00000000100;//-- st r1 ,r3 ,4
            instructions[20]=32'b100101_00001_00100_00000_00000001000;//-- st r1 ,r4 ,8
            instructions[21]=32'b100101_00001_00101_00000_00000001100;//-- st r1 ,r5 ,12
            instructions[22]=32'b100101_00001_00110_00000_00000010000;//-- st r1 ,r6 ,16
            instructions[23]=32'b100100_00001_01011_00000_00000000100;//-- ld r1 ,r11 ,4
            instructions[24]=32'b100101_00001_00111_00000_00000010100;//-- st r1 ,r7 ,20
            instructions[25]=32'b100101_00001_01000_00000_00000011000;//-- st r1 ,r8 ,24
            instructions[26]=32'b100101_00001_01001_00000_00000011100;//-- st r1 ,r9 ,28
            instructions[27]=32'b100101_00001_01010_00000_00000100000;//-- st r1 ,r10 ,32
            instructions[28]=32'b100101_00001_01011_00000_00000100100;//-- st r1 ,r11 ,36
            instructions[29]=32'b100000_00000_00001_00000_00000000011;//-- Addi r0 ,r1 ,3
            instructions[30]=32'b100000_00000_00100_00000_10000000000;//-- Addi r0 ,r4 ,1024
            instructions[31]=32'b100000_00000_00010_00000_00000000000;//-- Addi r0 ,r2 ,0
            instructions[32]=32'b100000_00000_00011_00000_00000000001;//-- Addi r0 ,r3 ,1 //////////////////
            instructions[33]=32'b100000_00000_01001_00000_00000000010;//-- Addi r0 ,r9 ,2 ////////++++
            instructions[34]=32'b001010_00011_01001_01000_00000000000;//-- sll r3 ,r9 ,r8
            instructions[35]=32'b000001_00100_01000_01000_00000000000;//-- Add r4 ,r8 ,r8
            instructions[36]=32'b100100_01000_00101_00000_00000000000;//-- ld r8 ,r5 ,0
            instructions[37]=32'b100100_01000_00110_11111_11111111100;//-- ld r8 ,r6 ,-4
            instructions[38]=32'b000011_00101_00110_01001_00000000000;//-- sub r5 ,r6 ,r9
            instructions[39]=32'b100000_00000_01010_10000_00000000000;//-- Addi r0 ,r10 ,0x8000
            instructions[40]=32'b100000_00000_01011_00000_00000010000;//-- Addi r0 ,r11 ,16
            instructions[41]=32'b001010_01010_01011_01010_00000000000;//-- sll r10 ,r11 ,r10
            instructions[42]=32'b000101_01001_01010_01001_00000000000;//-- And r9 ,r10 ,r9
            instructions[43]=32'b101000_01001_00000_00000_00000000010;//-- Bez r9 ,2
            instructions[44]=32'b100101_01000_00101_11111_11111111100;//-- st r8 ,r5 ,-4
            instructions[45]=32'b100101_01000_00110_00000_00000000000;//-- st r8 ,r6 ,0
            instructions[46]=32'b100000_00011_00011_00000_00000000001;//-- Addi r3 ,r3 ,1
            instructions[47]=32'b101001_00001_00011_11111_11111110001;//-- BNE r1 ,r3 ,-32 //////// 76 - 45 + 1
            instructions[48]=32'b100000_00010_00010_00000_00000000001;//-- Addi r2 ,r2 ,1;;
            instructions[49]=32'b101001_00001_00010_11111_11111101110;//-- BNE r1 ,r2 ,-39  //////////////////// 82 - 44 + 1
            instructions[50]=32'b100000_00000_00001_00000_10000000000;//-- Addi r0 ,r1 ,1024
            instructions[51]=32'b100100_00001_00010_00000_00000000000;//-- ld r1 ,r2 ,0
            instructions[52]=32'b100100_00001_00011_00000_00000000100;//-- ld r1 ,r3 ,4
            instructions[53]=32'b100100_00001_00100_00000_00000001000;//-- ld r1 ,r4 ,8
            instructions[54]=32'b100100_00001_00101_00000_00000001100;//-- ld r1 ,r5 ,12
            instructions[55]=32'b100100_00001_00110_00000_00000010000;//-- ld r1 ,r6 ,16
            instructions[56]=32'b100100_00001_00111_00000_00000010100;//-- ld r1 ,r7 ,20
            instructions[57]=32'b100100_00001_01000_00000_00000011000;//-- ld r1 ,r8 ,24
            instructions[58]=32'b100100_00001_01001_00000_00000011100;//-- ld r1 ,r9 ,28
            instructions[59]=32'b100100_00001_01010_00000_00000100000;//-- ld r1 ,r10 ,32
            instructions[60]=32'b100100_00001_01011_00000_00000100100;//-- ld r1 ,r11 ,36
            instructions[61]=32'b101010_00000_00000_11111_11111111111;//-- JMP -1
          end
     end
     always@(*)begin
          instruction = instructions[PC>>2];
     end
endmodule // Memory
