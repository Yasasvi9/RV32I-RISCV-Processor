`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/15/2025 05:53:46 PM
// Design Name: 
// Module Name: rv32i_regs
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


module rv32i_regs(
    // system clock and synchronous reset
    input clk,
    input reset,
    // inputs
    input logic [4:0] rs1_reg,
    input logic [4:0] rs2_reg,
    input wb_enable,
    input [4:0]wb_reg,
    input [31:0]wb_data,
    // outputs
    output logic [31:0] rs1_data,
    output logic [31:0] rs2_data
    );
    
    reg [31:0] register_file [31:0];

    always_ff @(posedge clk or posedge reset)
    begin
        if(reset)
        begin
            for(int i = 0; i < 32; i = i + 1)
                register_file[i] <= 32'b0;
        end
        else if(wb_enable && wb_reg != 0)
        begin
            register_file[wb_reg] <= wb_data;
        end
    end

    assign rs1_data = register_file[rs1_reg];
    assign rs2_data = register_file[rs2_reg];
    
endmodule
