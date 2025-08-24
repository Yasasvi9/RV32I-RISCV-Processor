`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/02/2025 04:49:04 PM
// Design Name: 
// Module Name: dual_port_ram_top
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


module dual_port_ram_top(
    // Clock
    input clk,
    // Instruction port (RO)
    input [31:2] i_addr,            
    output [31:0] i_rdata,
    // Data port (RW)
    input [31:0] d_addr,            // take in the entire data address so the shifter can use it for byte addressing
    output [31:0] d_rdata,
    input unsigned_flag,                 // signed/unsigned read flag
    input d_we,
    input [1:0] width,              // input [3:0] d_be,
    input [31:0] d_wdata
    );

    wire [31:0] d_s_wdata;               // shifted data output
    wire [31:0] r_data;                 // read data output before shifting
    

    // left shifter instantiation - Writes
    shifter left_shifter (
        .data_in(d_wdata),               // data to be shifted
        .shift_offset(d_addr[1:0]),      // shift amount
        .flag(unsigned_flag),             // signed/unsigned read flag
        .we(d_we),                      // write enable
        .width(width),                    // width of the data to be shifted (0 = byte, 1 = half-word, 2 = word)
        .data_out(d_s_wdata)             // shifted data
    );

    // right shifter instantiation - Reads
    shifter right_shifter (
        .data_in(r_data),               // data to be shifted
        .shift_offset(d_addr[1:0]),      // shift amount
        .flag(unsigned_flag),             // signed/unsigned read flag
        .we(d_we),                      // write enable
        .width(width),                    // width of the data to be shifted (0 = byte, 1 = half-word, 2 = word)
        .data_out(d_rdata)             // shifted data
    );

    // dual port RAM instantiation
    dual_port_ram bram (
        // Clock
        .clk(clk),
        // Instruction port (RO)
        .i_addr(i_addr),
        .i_rdata(i_rdata),
        // Data port (RW)
        .d_addr(d_addr[31:2]),            // leave out the bottom 2 bits of the address
        .d_rdata(r_data),
        .d_we(d_we),
        .d_be(d_be),
        .d_wdata(d_s_wdata)
    );

endmodule
