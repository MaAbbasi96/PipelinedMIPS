module condition_check(
    input [1:0] branch_type,
	input [31:0] val1, src2_val,
	output branch_taken);

    parameter BEZ = 3'd0, BNE = 3'd1, JMP = 3'd2, OTHER = 3'd3;


	assign branch_taken = (branch_type == BEZ && val1==0) ? 1:
	                      (branch_type == BNE && (val1 != src2_val)) ? 1:
                          (branch_type == JMP) ? 1 : 0;

endmodule
