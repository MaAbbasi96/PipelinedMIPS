module SignExt (
        input [15:0] immediate,
        output [31:0] immediate_ext
    );

    assign immediate_ext = (immediate[15])? {16'b1111111111111111,immediate}: immediate;
    
endmodule // SignExt
