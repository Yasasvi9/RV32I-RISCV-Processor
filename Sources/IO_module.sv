`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/25/2025 10:52:24 PM
// Design Name: 
// Module Name: IO_module
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


module IO_module(
    // system clock and synchronous reset
    input clk,
    input reset,
    // io interface
    input [31:0] io_addr,
    output reg [31:0] io_rdata,
    input io_we,
    input [3:0] io_be,
    input [31:0] io_wdata,
    // to RV31I_top
    output logic [31:0] leds_out      // LEDs output
    );

    // Internal storage for LED/SW/PB state
    logic [31:0] leds_reg;
    logic [31:0] sw_reg;
    logic [31:0] pb_reg;
    wire [3:0] offset = io_addr[3:0];

    // Write path: on io_we, update the targeted register
    always_ff @(posedge clk) 
    begin
        if (reset) 
        begin
            leds_reg <= 32'd0;
            sw_reg   <= 32'd0;
            pb_reg   <= 32'd0;
            io_rdata <= 32'd0; 
        end 
        else if (io_we) 
        begin
            case (offset)
                4'h0: 
                begin  // LEDs @ offset 0x0
                    if (io_be[0]) leds_reg[7:0] <= io_wdata[7:0];
                    if (io_be[1]) leds_reg[15:8] <= io_wdata[15:8];
                    if (io_be[2]) leds_reg[23:16] <= io_wdata[23:16];
                    if (io_be[3]) leds_reg[31:24] <= io_wdata[31:24];
                end
                4'h4: 
                begin  // Switches @ offset 0x4
                    if (io_be[0]) sw_reg[7:0] <= io_wdata[7:0];
                    if (io_be[1]) sw_reg[15:8] <= io_wdata[15:8];
                    if (io_be[2]) sw_reg[23:16] <= io_wdata[23:16];
                    if (io_be[3]) sw_reg[31:24] <= io_wdata[31:24];
                end
                4'h8: 
                begin  // Push-buttons @ offset 0x8
                    if (io_be[0]) pb_reg[7:0] <= io_wdata[7:0];
                    if (io_be[1]) pb_reg[15:8] <= io_wdata[15:8];
                    if (io_be[2]) pb_reg[23:16] <= io_wdata[23:16];
                    if (io_be[3]) pb_reg[31:24] <= io_wdata[31:24];
                end
                default:
                begin
                end
            endcase
        end
        // Read path: combinationally multiplex back the right register
        else
        begin 
            case (offset)
                4'h0: io_rdata = leds_reg;      // read LEDs
//                4'h4: io_rdata = switches_in;   // read physical switches
//                4'h8: io_rdata = pbs_in;        // read physical push-buttons
            endcase
        end
    end

    // Expose the LED output so top-level can hook it to board pins
    assign leds_out = leds_reg;

endmodule
