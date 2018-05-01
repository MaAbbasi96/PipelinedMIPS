module hazard_detection_unit (
        input [4:0] src1,
        input [4:0] src2,
        input [4:0] exe_dest,
        input exe_wb_en,
        input [4:0] mem_dest,
        input mem_wb_en,
        input one_input,
        input forward_enable,
        input MEM_R_EN,
        output hazard_detection
    );

    wire hazard_detection0, hazard_detection1;

    assign hazard_detection0 = (src1 == exe_dest && exe_wb_en)? 1:
                            (!one_input && src2 == exe_dest && exe_wb_en)? 1:
                            (src1 == mem_dest && mem_wb_en)? 1:
                            (!one_input && src2 == mem_dest && mem_wb_en)? 1:0;

    assign hazard_detection1 = (MEM_R_EN && (exe_dest == src1 || (exe_dest == src2 && !one_input )))? 1:0;

    assign hazard_detection = forward_enable ? hazard_detection1 : hazard_detection0;

endmodule
