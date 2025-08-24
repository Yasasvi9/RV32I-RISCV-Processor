`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/22/2025 05:24:13 PM
// Design Name: 
// Module Name: IF_stage
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module IF_stage(
    // system clock and synchronous reset
    input clk,
    input reset,
    // memory interface
    output [31:2] memif_addr,
    input [31:0] memif_data,
    // from id
    input jump_enable_in,
    input [31:0] jump_addr_in,
    // to id
    output reg [31:0] pc_out,
    output [31:0] iw_out, // note this was registered in the memory already
    // ebreak signal
    input ebreak
    );

    // program counter
    reg [31:0] pre_pc;
    reg [31:0] pc;
    parameter PC_RESET = 0;

    always_ff @(posedge clk)
    begin
        if (reset)
            pre_pc <= PC_RESET;
        else if (jump_enable_in)
            pre_pc <= jump_addr_in;
        else if (ebreak)
            pre_pc <= pre_pc; // keep the same value
        else
            pre_pc <= pre_pc + 4;
        pc <= pre_pc;
    end

    assign pc_out = pc;
    assign memif_addr = pre_pc[31:2];
    assign iw_out = memif_data;

endmodule
