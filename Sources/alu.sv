`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/30/2025 03:11:21 PM
// Design Name: 
// Module Name: alu
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

module alu (
        input [31:0] pc,
        input wire [31:0] rs1_data,             // Register 1 value
        input wire [31:0] rs2_data,             // Register 2 value
        input wire [31:0] imm,                  // Immediate value
        input wire [3:0] operand,               // ALU control signal
        input wire [2:0] instr_type,            // Instruction type
        output reg [31:0] alu_result            // ALU result
    );

    always_comb 
    begin
        case(operand)
            4'b0010: alu_result = rs1_data + (instr_type == 3'b000 ? rs2_data : imm);                         // ADD, ADDI, Store/Branch address calc
            4'b0110: alu_result = rs1_data - rs2_data;                                                        // SUB
            4'b0001: alu_result = rs1_data ^ (instr_type == 3'b000 ? rs2_data : imm);                         // XOR, XORI
            4'b0000: alu_result = rs1_data | (instr_type == 3'b000 ? rs2_data : imm);                         // OR, ORI
            4'b0011: alu_result = rs1_data & (instr_type == 3'b000 ? rs2_data : imm);                         // AND, ANDI
            4'b0100: alu_result = rs1_data << (instr_type == 3'b000 ? rs2_data[4:0] : imm[4:0]);              // SLL, SLLI
            4'b0111: alu_result = rs1_data >> (instr_type == 3'b000 ? rs2_data[4:0] : imm[4:0]);              // SRL, SRLI
            4'b1100: alu_result = $signed(rs1_data) >>> (instr_type == 3'b000 ? rs2_data[4:0] : imm[4:0]);    // SRA, SRAI
            4'b1010: alu_result = ($signed(rs1_data) < $signed(rs2_data)) ? 32'b1 : 32'b0;                    // SLT, SLTI
            4'b1011: alu_result = (rs1_data < rs2_data) ? 32'b1 : 32'b0;                                      // SLTU, SLTIU
            4'b0101: alu_result = imm;                                                                        // LUI
            4'b1000: alu_result = pc + imm;                                                                   // AUIPC
            default: alu_result = 32'b0;                                                                      // Default to zero
        endcase
    end

endmodule

