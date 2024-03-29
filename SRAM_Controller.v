module SRAM_Controller (
        input clk,
        input rst,
        //from memory stage
        input wr_en,
        input rd_en,
        input [31:0] address,
        input [31:0] writeData,
        //to next stage
        output reg [63:0] readData,
        //for freeze other stage
        output ready,
        inout [15:0] SRAM_DQ,				//	SRAM Data bus 16 Bits
        output [17:0] SRAM_ADDR,				//	SRAM Address bus 18 Bits
        output reg SRAM_UB_N,				//	SRAM High-byte Data Mask
        output reg SRAM_LB_N,				//	SRAM Low-byte Data Mask
        output reg SRAM_WE_N,				//	SRAM Write Enable
        output reg SRAM_CE_N,				//	SRAM Chip Enable
        output reg SRAM_OE_N
    );

    parameter WR0 = 3'd0, WR1 = 3'd1, WR2 = 3'd2, WR3 = 3'd3, WR4 = 3'd4;

    reg [2:0] ps, ns;
    wire [17:0] address1, address2, address3, address4, w_address1, w_address2;
    reg [17:0] SRAM_ADDR_reg;
    reg [15:0] SRAM_DQ_reg;

    wire [31:0] address_temp;
    assign address_temp = address - 1024;

    assign w_address1 =  {address_temp[18:2], 1'b0};
    assign w_address2 =  {address_temp[18:2], 1'b1};

    assign address1 = {address_temp[18:3], 2'b00};
    assign address2 = {address_temp[18:3], 2'b01};
    assign address3 = {address_temp[18:3], 2'b10};
    assign address4 = {address_temp[18:3], 2'b11};

    always @ ( ps ) begin
          if(ps == WR4) ns = WR0;
          else ns = ps + 1;
    end

    always @ ( posedge clk ) begin
        {SRAM_UB_N, SRAM_LB_N, SRAM_CE_N, SRAM_OE_N, SRAM_WE_N} = 5'd1;
          case(ps)
                WR0: begin
                        if(wr_en) begin SRAM_DQ_reg <= writeData[31:16]; SRAM_ADDR_reg <= w_address2; SRAM_WE_N <= 1'b0; end
                        else if(rd_en) begin readData[15:0] <= SRAM_DQ; end
                end
                WR1: begin
                        if(wr_en) begin SRAM_DQ_reg <= writeData[15:0]; SRAM_ADDR_reg <= w_address1; SRAM_WE_N <= 1'b0; end
                        else if(rd_en) begin readData[31:16] <= SRAM_DQ; end
                end
                WR2: begin
                        if(rd_en) begin readData[47:32] <= SRAM_DQ; end
                end
                WR3: begin
                        if(rd_en) begin readData[63:48] <= SRAM_DQ; end
                end
          endcase
    end

    always @ ( posedge clk ) begin
         if(rst | (!wr_en & !rd_en))
               ps <= WR0;
          else
               ps <= ns;
    end

    assign SRAM_ADDR = (wr_en) ? SRAM_ADDR_reg : (ps == WR0) ? address1 : (ps == WR1) ? address2 : (ps == WR2) ? address3 : (ps == WR3) ? address4 : 16'b0;
    assign SRAM_DQ = (wr_en) ? SRAM_DQ_reg : 16'bzzzzzzzzzzzzzzzz;
    assign ready = ((!wr_en & !rd_en) | ((ps == WR0 || ps == WR3 || ps == WR2 || ps == WR1) && (wr_en | rd_en))) ? 0 : 1;

endmodule // SRAM_Controller
