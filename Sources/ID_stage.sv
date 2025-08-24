`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/22/2025 05:26:28 PM
// Design Name: 
// Module Name: ID_stage
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


module ID_stage(
    // system clock and synchronous reset
    input clk,
    input reset,
    // from if
    input [31:0] pc_in,
    input [31:0] iw_in,
    // to if
    output jump_enable_out,
    output [31:0] jump_addr_out,
    // register interface
    output [4:0] regif_rs1_reg,
    output [4:0] regif_rs2_reg,
    input [31:0] regif_rs1_data,
    input [31:0] regif_rs2_data,
    // to ex
    output reg [31:0] pc_out,
    output reg [31:0] iw_out,
    output reg [4:0] wb_reg_out,
    output reg wb_enable_out,
    output reg mem_we_out,
    output [31:0] rs1_data_out,
    output [31:0] rs2_data_out,
    output reg ebreak_out,
    // data hazard: df from ex
    input df_ex_enable,
    input [4:0] df_ex_reg,
    input [31:0] df_ex_data,
    // data hazard: df from mem
    input df_mem_enable,
    input [4:0] df_mem_reg,
    input [31:0] df_mem_data,
    // data hazard: df from wb
    input df_wb_enable,
    input [4:0] df_wb_reg,
    input [31:0] df_wb_data
    );

    localparam EBREAK = 32'h00100073; // Ebreak instruction
    localparam LOAD = 7'b0000011; // Load instruction opcode
    localparam STORE = 7'b0100011; // Store instruction opcode
    
    wire is_load = (iw_in[6:0] == LOAD); // Check if the instruction is a load
    wire is_store = (iw_in[6:0] == STORE); // Check if the instruction is a store
    wire is_ebreak = (iw_out == EBREAK); // Check if the instruction is EBREAK

    reg ebreak;

    reg [4:0] wb_reg;
    reg wb_enable;

    // register file interface
    wire [4:0] rs1_field = iw_in[19:15]; // rs1 field
    wire [4:0] rs2_field = iw_in[24:20]; // rs2 field
    wire rs1_valid = ((iw_in[6:0] == 7'b1110011) || (iw_in[6:0] == 7'b0110111) || (iw_in[6:0] == 7'b0010111) || (iw_in[6:0] == 7'b1101111)) ? 1'b0 : 1'b1; // Except Ebreak, Ecall, U-type and J-type instructions
    wire rs2_valid = ((iw_in[6:0] == 7'b0110011) || (iw_in[6:0] == 7'b0100011) || (iw_in[6:0] == 7'b1100011)) ? 1'b1 : 1'b0; // R-type and S-type and B-type instructions

    assign regif_rs1_reg = rs1_valid ? rs1_field : 5'b0;
    assign regif_rs2_reg = rs2_valid ? rs2_field : 5'b0; 

    // Write back signal generation
    wire [4:0] rd_field = iw_in[11:7]; // rd field
    wire rd_valid = ((iw_in[6:0] == 7'b1110011) || (iw_in[6:0] == 7'b0100011) || (iw_in[6:0] == 7'b1100011)) ? 1'b0 : 1'b1; // Except S-type and B-type instructions 

    // Data forwarding logic
    reg[31:0] rs1_data_buffer;
    reg[31:0] rs2_data_buffer;
    logic [31:0] rs1_data;
    logic [31:0] rs2_data;
    
    always_comb
    begin
        if (df_ex_enable && (df_ex_reg == rs1_field))
            rs1_data = df_ex_data;
        else if (df_mem_enable && (df_mem_reg == rs1_field))
            rs1_data = df_mem_data;
        else if (df_wb_enable && (df_wb_reg == rs1_field))
            rs1_data = df_wb_data;
        else 
            rs1_data = regif_rs1_data;

        if (df_ex_enable && (df_ex_reg == rs2_field))
            rs2_data = df_ex_data;
        else if (df_mem_enable && (df_mem_reg == rs2_field))
            rs2_data = df_mem_data;
        else if (df_wb_enable && (df_wb_reg == rs2_field))
            rs2_data = df_wb_data;
        else
            rs2_data = regif_rs2_data;
    end

    always_ff @(posedge clk or posedge reset)
    begin
        if (reset)
        begin
            rs1_data_buffer <= 0;
            rs2_data_buffer <= 0;
        end
        else
        begin
            rs1_data_buffer <= rs1_data;
            rs2_data_buffer <= rs2_data;
        end
    end

    // output assignments to EX stage
    assign wb_reg = rd_valid ? rd_field : 5'b0;
    assign wb_enable = rd_valid || is_load;
    assign rs1_data_out = rs1_data_buffer;
    assign rs2_data_out = rs2_data_buffer;

    // Branch and Jump control logic
    wire [6:0] opcode = iw_in[6:0];
    wire [2:0] funct3 = iw_in[14:12];
    wire jump_enable;
    reg jump_enable_buffer;

    assign jump_enable = ((opcode == 7'b1101111) || // J-type
                             (opcode == 7'b1100011 && 
                              ((funct3 == 3'b000 && $signed(rs1_data) == $signed(rs2_data)) ||    // BEQ
                               (funct3 == 3'b001 && $signed(rs1_data) != $signed(rs2_data)) ||    // BNE
                               (funct3 == 3'b100 && $signed(rs1_data) < $signed(rs2_data)) ||     // BLT
                               (funct3 == 3'b101 && $signed(rs1_data) >= $signed(rs2_data)) ||    // BGE
                               (funct3 == 3'b110 && rs1_data < rs2_data) ||                       // BLTU
                               (funct3 == 3'b111 && rs1_data >= rs2_data))) ||                    // BGEU
                             (opcode == 7'b1100111)); // I-type for jalr
    assign jump_enable_out = ~jump_enable_buffer && jump_enable;
    assign jump_addr_out = (opcode == 7'b1101111) ? pc_in + {{12{iw_in[31]}}, iw_in[19:12], iw_in[20], iw_in[30:21], 1'b0} :        // J-type
                            (opcode == 7'b1100011) ? pc_in + {{20{iw_in[31]}}, iw_in[7], iw_in[30:25], iw_in[11:8], 1'b0} :         // B-type
                            (opcode == 7'b1100111) ? ((rs1_data + {{20{iw_in[31]}}, iw_in[31:20]}) & 32'hFFFFFFFE) : 32'h0;  // I-type for jalr

    always_ff @(posedge clk or posedge reset)
    begin
        if (reset)
        begin
            pc_out <= 0;
            iw_out <= 0;
            wb_reg_out <= 0;
            wb_enable_out <= 0;
            mem_we_out <= 0; // Default to no memory writeback
            jump_enable_buffer <= 1'b0;
            ebreak <= 1'b0;
        end
        else
        begin
            pc_out <= pc_in;
            jump_enable_buffer <= jump_enable_out;
            iw_out <= jump_enable_buffer ? 32'h00000013 : iw_in; // NOP instruction
            wb_reg_out <= wb_reg;
            wb_enable_out <= wb_enable;
            mem_we_out <= is_store; 
            ebreak <= is_ebreak; // Set ebreak if the instruction is EBREAK
        end 
    end

    assign ebreak_out = ebreak;
    assign jump_enable_buffer_out = jump_enable_buffer;

endmodule
