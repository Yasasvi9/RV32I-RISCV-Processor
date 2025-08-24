`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/14/2025 10:38:48 PM
// Design Name: 
// Module Name: byte_enable_gen
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


module byte_enable_gen(
    input [1:0] addr_offset,           // address offset
    input [1:0] width,                 // width of the data to be shifted (0 = byte, 1 = half-word, 2 = word)
    output reg [3:0] d_be              // byte enables
    );

    always_comb 
    begin 
        case (width)
            2'b00: // Store Byte
                case (addr_offset)
                    2'b00: d_be = 4'b0001;      // Byte in [7:0]
                    2'b01: d_be = 4'b0010;      // Byte in [15:8]
                    2'b10: d_be = 4'b0100;      // Byte in [23:16]
                    2'b11: d_be = 4'b1000;      // Byte in [31:24]
                endcase

            2'b01: // Store Half-Word
                // For a half-word, offset[1] indicates whether to shift by 0 or 16 bits
                case (addr_offset[1])
                    1'b0: d_be = 4'b0011;      // HW in [15:0]
                    1'b1: d_be = 4'b1100;      // HW in [31:16]
                endcase

            2'b10: // Store Word
                // No shift needed; entire 32-bit value goes straight to memory
                d_be = 4'b1111;

            default:
                d_be = 4'b0000; // Safe default
        endcase      
    end

endmodule
