`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/22/2025 05:46:35 PM
// Design Name: 
// Module Name: MEM_stage
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


module MEM_stage(
    // system clock and synchronous reset
    input clk,
    input reset,
    // to id
    output df_mem_enable,
    output [4:0] df_mem_reg,
    output [31:0] df_mem_data,
    // from ex
    input [31:0] pc_in,
    input [31:0] iw_in,
    input [31:0] alu_in, // ALU result from EX stage
    input [31:0] rs2_data_in, // Data from RS2 in EX stage
    input [4:0] wb_reg_in,
    input wb_enable_in,
    input mem_we_in,
    // memory interface
    output [31:2] memif_addr,
    input [31:0] memif_rdata,
    output memif_we,
    output [3:0] memif_be,
    output [31:0] memif_wdata,
    // io interface
    output [31:2] io_addr,
    input [31:0] io_rdata,
    output io_we,
    output [3:0] io_be,
    output [31:0] io_wdata,
    // to wb
    output reg [31:0] pc_out,
    output reg [31:0] iw_out,
    output reg [31:0] alu_out,
    output reg [31:0] memif_rdata_out,
    output reg [31:0] io_rdata_out,
    output reg [4:0] wb_reg_out,
    output reg wb_enable_out,
    output reg [1:0] wb_src_out
    );

    localparam LOAD = 7'b0000011; // Load instruction opcode
    localparam STORE = 7'b0100011; // Store instruction opcode
    localparam WB_ALU = 2'b00; // Write-back from ALU
    localparam WB_RAM = 2'b01; // Write-back from memory
    localparam WB_IO = 2'b10; // Write-back from IO

    wire [3:0] d_be;                    // byte enables

    wire wb_src = (wb_enable_in ? 
                    (iw_in[6:0] == LOAD ? 
                        (alu_in[31] ? WB_IO : WB_RAM) : 
                    WB_ALU) : // Store instruction
                WB_ALU);

    // Registering the inputs
    always_ff @(posedge clk or posedge reset)
    begin
        if (reset)
        begin
            pc_out <= 0;
            iw_out <= 0;
            alu_out <= 0;
            wb_reg_out <= 0;
            wb_enable_out <= 0;
            wb_src_out <= WB_ALU; // Default to ALU write-back
        end
        else
        begin
            pc_out <= pc_in;
            iw_out <= iw_in;
            alu_out <= alu_in;
            wb_reg_out <= wb_reg_in;
            wb_enable_out <= wb_enable_in;
            wb_src_out <= wb_src;
        end
    end

    // Data forwarding signals
    assign df_mem_enable = wb_enable_out;   
    assign df_mem_reg = wb_reg_out;
    assign df_mem_data = alu_out;

    // Byte Enables Generation
    byte_enable_gen be_gen (
        .addr_offset(alu_in[1:0]),                                                  // address offset
        .width(iw_in[13:12]),                                                       // width of the data (0 = byte, 1 = half-word, 2 = word)
        .d_be(d_be)                                                                 // byte enables
    );

    assign memif_addr = alu_in[31:2];                                               // Address for memory access
    assign memif_wdata = rs2_data_in;                                               // Data to be written to memory
    assign memif_we = mem_we_in && !alu_in[31];                                     // Write enable for store instructions
    assign memif_be = d_be;                                                         // Byte enables for memory access
    assign memif_rdata_out = memif_rdata;                                           // Data read from memory
    assign io_addr = alu_in[31:2];                                                  // Address for IO access
    assign io_wdata = rs2_data_in;                                                  // Data to be written to IO
    assign io_we = mem_we_in && alu_in[31];                                         // Write enable for IO access
    assign io_be = d_be;                                                            // Byte enables for IO access
    assign io_rdata_out = io_rdata;                                                 // Data read from IO

endmodule
