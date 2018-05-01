module MEM_Stage_reg(
        input clk,
        input rst,
        input WB_en_in,
        //MEM_Signals
        input MEM_R_EN_in,
        //memory Address
        input [31:0] ALU_result_in,

        input [31:0] MEM_read_value_in,
        input [4:0] Dest_in,
        input freeze,

        output reg WB_en,
        //MEM_Signals
        output reg MEM_R_EN,
        //memory Address
        output reg [31:0] ALU_result,

        output reg[31:0] MEM_read_value,
        output reg[4:0] Dest
    );

    always @(posedge clk) begin
        if(rst) begin
            WB_en <= 1'b0;
            MEM_R_EN <= 1'b0;
            ALU_result <= 32'b0;
            MEM_read_value <= 32'b0;
            Dest <= 5'b0;
        end

        else if(!freeze) begin
            WB_en <= WB_en_in;
            MEM_R_EN <= MEM_R_EN_in;
            ALU_result <= ALU_result_in;
            MEM_read_value <= MEM_read_value_in;
            Dest <= Dest_in;
        end
    end

endmodule //MEM_Stage_reg
