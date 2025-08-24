`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/29/2025 05:10:10 PM
// Design Name: 
// Module Name: EX_stage
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


    module EX_stage(
    // system clock and synchronous reset
    input clk,
    input reset,
    // from id
    input [31:0] pc_in,
    input [31:0] iw_in,
    input [31:0] rs1_data_in,
    input [31:0] rs2_data_in,
    input [4:0] wb_reg_in,
    input wb_enable_in,
    input mem_we_in,
    // to id
    output df_ex_enable,
    output [4:0] df_ex_reg,
    output [31:0] df_ex_data,
    // to mem
    output reg [31:0] pc_out,
    output reg [31:0] iw_out,
    output reg [31:0] alu_out,
    output reg [31:0] rs2_data_out,
    output reg [4:0] wb_reg_out,
    output reg wb_enable_out,
    output reg mem_we_out
    );
    
    reg [31:0] result;

    // Registering the inputs
    always_ff @(posedge clk or posedge reset)
    begin
        if (reset)
        begin
            pc_out <= 0;
            iw_out <= 0;
            alu_out <= 0;
            rs2_data_out <= 0;
            wb_reg_out <= 0;
            wb_enable_out <= 0;
            mem_we_out <= 0;
        end
        else
        begin
            pc_out <= pc_in;
            iw_out <= iw_in;
            alu_out <= result;
            rs2_data_out <= rs2_data_in;
            wb_reg_out <= wb_reg_in;
            wb_enable_out <= wb_enable_in;
            mem_we_out <= mem_we_in;
        end
    end
    
    // Instruction Word Decoding
    reg [6:0] opcode;
    reg [4:0] rd;
    reg [2:0] func3;
    reg [4:0] rs1;
    reg [4:0] rs2;
    reg [6:0] func7;
    
   always_comb
   begin
       opcode = iw_in[6:0];
       rd     = iw_in[11:7];
       func3  = iw_in[14:12];
       rs1    = iw_in[19:15];
       rs2    = iw_in[24:20];
       func7  = iw_in[31:25];      
   end

    // Immediate / Shamt Extraction with sign extension 
    reg [31:0] imm_I, imm_S, imm_B, imm_U, imm_J, imm;

    always_comb 
    begin
        imm_I = {{20{iw_in[31]}}, iw_in[31:20]};
        imm_S = {{20{iw_in[31]}}, iw_in[31:25], iw_in[11:7]};
        imm_B = {{19{iw_in[31]}}, iw_in[31], iw_in[7], iw_in[30:25], iw_in[11:8], 1'b0};
        imm_U = {iw_in[31:12], 12'b0};
        imm_J = {{11{iw_in[31]}}, iw_in[31], iw_in[19:12], iw_in[20], iw_in[30:21], 1'b0};
    end
    
    // Instruction Type Decoding 
    typedef enum logic [2:0]
    {
        R_TYPE, I_TYPE, S_TYPE, B_TYPE, U_TYPE, J_TYPE, EBREAK
    } instr_type_t;
    
    instr_type_t instr_type;
    
    always_comb
    begin
        case(opcode)
            7'b0110011: instr_type = R_TYPE;                            // R-Type
            7'b0010011, 7'b0000011, 7'b1100111: instr_type = I_TYPE;    // I-Type (ALU, Load, JALR)
            7'b0100011: instr_type = S_TYPE;                            // S-Type (Store)
            7'b1100011: instr_type = B_TYPE;                            // B-Type (Branch)
            7'b0110111, 7'b0010111: instr_type = U_TYPE;                // U-Type (LUI, AUIPC)
            7'b1101111: instr_type = J_TYPE;                            // J-Type (JAL)
            7'b1110011: instr_type = EBREAK;                            // EBREAK
            default: instr_type = I_TYPE;
        endcase
    end
    
    always_comb
    begin
        // Select the correct immediate value based on instr_type
        case(instr_type)
            I_TYPE: imm = imm_I;
            S_TYPE: imm = imm_S;
            B_TYPE: imm = imm_B;
            U_TYPE: imm = imm_U;
            J_TYPE: imm = imm_J;
            default: imm = 32'b0;
        endcase
    end
    
    // Determine Instruction
    reg [3:0] operand;
    
    always_comb
    begin
        case(instr_type)
            R_TYPE:
            begin
                case(func3)
                    3'b000: operand = (func7 == 7'b0000000) ? 4'b0010 : 4'b0110;    // ADD or SUB
                    3'b001: operand = 4'b0100;                                      // SLL
                    3'b010: operand = 4'b1010;                                      // SLT
                    3'b011: operand = 4'b1011;                                      // SLTU
                    3'b100: operand = 4'b0001;                                      // XOR
                    3'b101: operand = (func7 == 7'b0000000) ? 4'b0111 : 4'b1100;    // SRL or SRA
                    3'b110: operand = 4'b0000;                                      // OR
                    3'b111: operand = 4'b0011;                                      // AND 
                endcase
            end
            I_TYPE:
            begin
                case(func3)
                    3'b000: operand = 4'b0010;                                      // ADDI
                    3'b010: operand = 4'b1010;                                      // SLTI
                    3'b011: operand = 4'b1011;                                      // SLTIU
                    3'b100: operand = 4'b0001;                                      // XORI
                    3'b110: operand = 4'b0000;                                      // ORI
                    3'b111: operand = 4'b0011;                                      // ANDI
                    3'b001: operand = 4'b0100;                                      // SLLI
                    3'b101: operand = (func7 == 7'b0000000) ? 4'b0111 : 4'b1100;    // SRLI or SRAI
                endcase  
            end
            S_TYPE: operand = 4'b0010;                                              // Address calculations for Store/Branch
            U_TYPE: operand = (opcode == 7'b0010111) ? 4'b1000 : 4'b0101;           // LUI, AUIPC
            J_TYPE: operand = 4'b0111;                                              // JAL *** needs to get a different operand which doesnt conflict with SRA  (maybe 4'b1111)
            B_TYPE: operand = 4'b1111;                                              // Branch operations (BEQ, BNE, BLT, BGE, BLTU, BGEU)
            default: operand = 4'b0000;                                             // *** need to get a case for JALR and JAL to perform rd <- pc + 4
        endcase
    end
    
    alu alu_inst(.pc(pc_in), .rs1_data(rs1_data_in), .rs2_data(rs2_data_in), .imm(imm), .operand(operand), .instr_type(instr_type), .alu_result(result));
    
    // Data forwarding signals
    assign df_ex_enable = wb_enable_in;
    assign df_ex_reg = wb_reg_in;
    assign df_ex_data = result;
    
endmodule