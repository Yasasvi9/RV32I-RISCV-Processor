`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/14/2025 07:51:04 PM
// Design Name: 
// Module Name: shifter
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


module shifter(
    input [31:0] data_in,               // data to be shifted
    input [1:0] shift_offset,           // shift amount
    input flag,                     // signed/unsigned read flag
    input we,                           // write enable
    input [1:0] width,                  // width of the data to be shifted (0 = byte, 1 = half-word, 2 = word)
    output reg [31:0] data_out          // shifted data
    );

    // Shifter function to store into memory
    function automatic logic [31:0] left_shift(
        input logic [31:0] data,
        input logic [1:0]  addr_offset,
        input logic [1:0]  width
    );
        logic [31:0] result;
        begin
            case (width)
                2'b00: // Store Byte
                    case (addr_offset)
                        2'b00: result = {24'b0, data[7:0]};      // Byte in [7:0]
                        2'b01: result = {16'b0, data[7:0], 8'b0}; // Byte in [15:8]
                        2'b10: result = {8'b0, data[7:0], 16'b0}; // Byte in [23:16]
                        2'b11: result = {data[7:0], 24'b0};       // Byte in [31:24]
                    endcase

                2'b01: // Store Half-Word
                    // For a half-word, offset[1] indicates whether to shift by 0 or 16 bits
                    case (addr_offset[1])
                        1'b0: result = {16'b0, data[15:0]};      // HW in [15:0]
                        1'b1: result = {data[15:0], 16'b0};       // HW in [31:16]
                    endcase

                2'b10: // Store Word
                    // No shift needed; entire 32-bit value goes straight to memory
                    result = data;

                default:
                    result = 32'h00000000; // Safe default
            endcase
            return result;
        end
    endfunction

    // Shifter function to read from memory
    function automatic logic [31:0] right_shift(
        input logic [31:0] data,
        input logic [1:0]  addr_offset,
        input logic [1:0]  width,
        input logic flag
    );
        logic [31:0] result;
        begin
            case (width)
                2'b00: // Read Byte
                    case (addr_offset)
                        2'b00: result = flag ? {24'b0, data[7:0]} : {{24{data[7]}}, data[7:0]};      // Byte in [7:0]
                        2'b01: result = flag ? {24'b0, data[15:8]} : {{24{data[15]}}, data[15:8]};   // Byte in [15:8]
                        2'b10: result = flag ? {24'b0, data[23:16]} : {{24{data[23]}}, data[23:16]}; // Byte in [23:16]
                        2'b11: result = flag ? {24'b0, data[31:24]} : {{24{data[31]}}, data[31:24]}; // Byte in [31:24]
                    endcase

                2'b01: // Read Half-Word
                    // For a half-word, offset[1] indicates whether to shift by 0 or 16 bits
                    case (addr_offset[1])
                        1'b0: result = flag ? {16'b0, data[15:0]} : {{16{data[15]}}, data[15:0]};    // HW in [15:0]
                        1'b1: result = flag ? {16'b0, data[31:16]} : {{16{data[31]}}, data[31:16]};  // HW in [31:16]
                    endcase

                2'b10: // Read Word
                    // No shift needed; entire 32-bit value goes straight to memory
                    result = data;

                default:
                    result = 32'h00000000; // Safe default
            endcase
            return result;
        end
    endfunction

    // Combinational logic to assign the shifted value
    assign data_out = we ? left_shift(data_in, shift_offset, width) : right_shift(data_in, shift_offset, width, flag);

endmodule
