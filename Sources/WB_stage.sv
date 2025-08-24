`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/22/2025 05:47:39 PM
// Design Name: 
// Module Name: WB_stage
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


module WB_stage(
    // system clock and synchronous reset
    input clk,
    input reset,
    // to id
    output df_wb_enable,
    output [4:0] df_wb_reg,
    output [31:0] df_wb_data,
    // from mem
    input [31:0] pc_in,
    input [31:0] iw_in,
    input [31:0] alu_in,
    input [31:0] memif_rdata_in,
    input [31:0] io_rdata_in,
    input [4:0] wb_reg_in,
    input wb_enable_in,
    input [1:0] wb_src_in, // Source for write-back data (ALU, MEM, IO)
    // todo: Out of WB (for ILA purposes) (remove later)
    output reg [31:0] pc_out,
    output reg [31:0] iw_out,
    // register interface
    output reg regif_wb_enable,
    output reg [4:0] regif_wb_reg,
    output reg [31:0] regif_wb_data    
    );

    localparam WB_SRC_ALU = 2'b00;
    localparam WB_SRC_MEM = 2'b01;
    localparam WB_SRC_IO = 2'b10;

    logic [31:0] wb_data; // Data to be written back to the register file
    always_comb begin
    case (wb_src_in)
        WB_SRC_ALU: wb_data = alu_in;            // from EX
        WB_SRC_MEM: wb_data = memif_rdata_in;    // from RAM
        WB_SRC_IO : wb_data = io_rdata_in;       // from I/O
        default: wb_data = alu_in;
    endcase
    end

    // Registering the inputs
    always_ff @(posedge clk or posedge reset)
    begin
        if (reset)
        begin
            pc_out <= 0;
            iw_out <= 0;
            regif_wb_reg <= 0;
            regif_wb_enable <= 0;
            regif_wb_data <= 0;
        end
        else
        begin
            pc_out <= pc_in;
            iw_out <= iw_in;
            regif_wb_reg <= wb_reg_in;
            regif_wb_enable <= wb_enable_in;
            regif_wb_data <= wb_data;
        end
    end
    
    // Data forwarding signals
    assign df_wb_enable = regif_wb_enable;
    assign df_wb_reg = regif_wb_reg;
    assign df_wb_data = regif_wb_data;

endmodule
