`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/29/2025 04:53:58 PM
// Design Name: 
// Module Name: RV32I_top
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


module RV32I_top(
    input CLK100,           // 100 MHz clock input
    output [9:0] LED,       // RGB1, RGB0, LED 9..0 placed from left to right
    output [2:0] RGB0,      
    output [2:0] RGB1,
    output [3:0] SS_ANODE,   // Anodes 3..0 placed from left to right
    output [7:0] SS_CATHODE, // Bit order: DP, G, F, E, D, C, B, A
    input [11:0] SW,         // SWs 11..0 placed from left to right
    input [3:0] PB,          // PBs 3..0 placed from left to right
    inout [23:0] GPIO,       // PMODA-C 1P, 1N, ... 3P, 3N order
    output [3:0] SERVO,      // Servo outputs
    output PDM_SPEAKER,      // PDM signals for mic and speaker
    input PDM_MIC_DATA,      
    output PDM_MIC_CLK,
    output ESP32_UART1_TXD,  // WiFi/Bluetooth serial interface 1
    input ESP32_UART1_RXD,
    output IMU_SCLK,         // IMU spi clk
    output IMU_SDI,          // IMU spi data input
    input IMU_SDO_AG,        // IMU spi data output (accel/gyro)
    input IMU_SDO_M,         // IMU spi data output (mag)
    output IMU_CS_AG,        // IMU cs (accel/gyro) 
    output IMU_CS_M,         // IMU cs (mag)
    input IMU_DRDY_M,        // IMU data ready (mag)
    input IMU_INT1_AG,       // IMU interrupt (accel/gyro)
    input IMU_INT_M,         // IMU interrupt (mag)
    output IMU_DEN_AG        // IMU data enable (accel/gyro)
    );
    
    wire [31:0] leds_data;

    // Terminate all of the unused outputs or i/o's
    assign LED = leds_data[9:0];
    assign RGB0 = leds_data[12:10];
    assign RGB1 = leds_data[15:13];
    assign SS_ANODE = 4'b0000;
    assign SS_CATHODE = 8'b11111111;
    assign GPIO = 24'bzzzzzzzzzzzzzzzzzzzzzzzz;
    assign SERVO = 4'b0000;
    assign PDM_SPEAKER = 1'b0;
    assign PDM_MIC_CLK = 1'b0;
    assign ESP32_UART1_TXD = 1'b0;
    assign IMU_SCLK = 1'b0;
    assign IMU_SDI = 1'b0;
    assign IMU_CS_AG = 1'b1;
    assign IMU_CS_M = 1'b1;
    assign IMU_DEN_AG = 1'b0;

    // renaming the clk signal
    wire clk = CLK100;
    
    // Reset Logic
    reg reset, reset_buffered;
    reg reset_in = PB[0];
    
    // Debouncer Logic
    reg [15:0] debouncer;
    reg debounce_reset;
    
    always_ff @ (posedge clk)
    begin
        debouncer <= {debouncer[14:0], reset_in};
        if(debouncer)
            debounce_reset <= 1'b1;
        else if(~debouncer)
            debounce_reset <= 1'b0;
    end
    
    // Metastability for the Reset Button
    always_ff @ (posedge clk)
    begin  
        reset_buffered <= debounce_reset;
        reset <= reset_buffered;
    end
    
    // Temporary Inputs for Testing
    // reg [31:0] pc_in = 32'b0;
    reg [31:0] iw_in = 32'b00000000000100000000000010110011;
    wire [31:0] rs1_data_in;
    wire [31:0] rs2_data_in;
    
    wire [31:0] pc_IF;
    wire [31:0] pc_ID;
    wire [31:0] pc_EX;
    wire [31:0] pc_MEM;
    wire [31:0] pc_WB;
    wire [31:0] iw_IF;
    wire [31:0] iw_ID;
    wire [31:0] iw_EX;
    wire [31:0] iw_MEM;
    wire [31:0] iw_WB;  
    wire [31:0] alu_out_EX;
    wire [31:0] alu_out_MEM; 
    wire [4:0] wb_reg_out_ID;
    wire [4:0] wb_reg_out_EX;
    wire [4:0] wb_reg_out_MEM;
    wire [4:0] wb_reg_out_WB;
    wire wb_en_ID;
    wire wb_en_EX;
    wire wb_en_MEM;
    wire wb_en_WB;
    wire [31:0] wb_data; // output from the ALU after WB stage
    wire [31:0] d_rdata;
    wire [31:0] i_rdata;
    wire [31:0] rs1_data_out; // output from the register file for rs1
    wire [31:0] rs2_data_out; // output from the register file for rs2
    wire [31:0] rs1_data_ID; // temporary input for rs1 data
    wire [31:0] rs2_data_ID; // temporary input for rs2 data
    wire ebreak;
    wire df_ex_enable;
    wire [4:0] df_ex_reg;
    wire [31:0] df_ex_data;
    wire df_mem_enable;
    wire [4:0] df_mem_reg;
    wire [31:0] df_mem_data;
    wire df_wb_enable;
    wire [4:0] df_wb_reg;
    wire [31:0] df_wb_data;
    wire jump_enable;
    wire [31:0] jump_addr;
    wire [31:2] d_address; // Memory interface address
    wire [31:0] d_wdata; // Memory interface write data
    wire d_we; // Memory interface write enable
    wire [3:0] d_be; // Memory interface byte enable
    wire [31:0] d_rdata; // Data to be written to memory
    wire [31:2] io_addr; // IO interface address
    wire [31:0] io_rdata; // IO interface read data
    wire io_we; // IO interface write enable
    wire [3:0] io_be; // IO interface byte enable
    wire [31:0] io_wdata; // IO interface write data
    wire [31:0] rs2_data_EX; // Data from RS2 in EX stage
    wire mem_we_ID; // Memory write enable from EX stage
    wire mem_we_EX; // Memory write enable from EX stage
    wire [31:0] mem_rdata; // Memory read data from MEM stage
    wire [31:0] io_rdata_mem; // IO read data from MEM stage
    wire [1:0] wb_src_out_MEM; // Write-back source from MEM stage

    // IF Stage instantiation
    IF_stage if_stage(.clk(clk), .reset(reset), .memif_addr(i_addr), .memif_data(i_rdata), .jump_enable_in(jump_enable), .jump_addr_in(jump_addr), .pc_out(pc_IF), .iw_out(iw_IF), .ebreak(ebreak));

    // ID Stage instantiation
    ID_stage id_stage(.clk(clk), .reset(reset), .pc_in(pc_IF), .iw_in(iw_IF), .jump_enable_out(jump_enable), .jump_addr_out(jump_addr), .regif_rs1_reg(rs1_reg), .regif_rs2_reg(rs2_reg), .regif_rs1_data(rs1_data_out), .regif_rs2_data(rs2_data_out), .pc_out(pc_ID), .iw_out(iw_ID), .wb_reg_out(wb_reg_out_ID), .wb_enable_out(wb_en_ID), .mem_we_out(mem_we_ID), .rs1_data_out(rs1_data_ID), .rs2_data_out(rs2_data_ID), .ebreak_out(ebreak), .df_ex_enable(df_ex_enable), .df_ex_reg(df_ex_reg), .df_ex_data(df_ex_data), .df_mem_enable(df_mem_enable), .df_mem_reg(df_mem_reg), .df_mem_data(df_mem_data), .df_wb_enable(df_wb_enable), .df_wb_reg(df_wb_reg), .df_wb_data(df_wb_data));
       
    // EX Stage instantiation
    EX_stage ex_stage(.clk(clk), .reset(reset), .pc_in(pc_ID), .iw_in(iw_ID), .rs1_data_in(rs1_data_ID), .rs2_data_in(rs2_data_ID), .wb_reg_in(wb_reg_out_ID), .wb_enable_in(wb_en_ID), .mem_we_in(mem_we_ID), .df_ex_enable(df_ex_enable), .df_ex_reg(df_ex_reg), .df_ex_data(df_ex_data) , .pc_out(pc_EX), .iw_out(iw_EX), .alu_out(alu_out_EX), .rs2_data_out(rs2_data_EX), .wb_reg_out(wb_reg_out_EX), .wb_enable_out(wb_en_EX), .mem_we_out(mem_we_EX));

    // MEM Stage instantiation
    MEM_stage mem_stage(.clk(clk), .reset(reset), .df_mem_enable(df_mem_enable), .df_mem_reg(df_mem_reg), .df_mem_data(df_mem_data), .pc_in(pc_EX), .iw_in(iw_EX), .alu_in(alu_out_EX), .rs2_data_in(rs2_data_EX), .wb_reg_in(wb_reg_out_EX), .wb_enable_in(wb_en_EX), .mem_we_in(mem_we_EX), .memif_addr(d_address), .memif_rdata(d_rdata), .memif_we(d_we), .memif_be(d_be), .memif_wdata(d_wdata), .io_addr(io_addr), .io_rdata(io_rdata), .io_we(io_we), .io_be(io_be), .io_wdata(io_wdata), .pc_out(pc_MEM), .iw_out(iw_MEM), .alu_out(alu_out_MEM), .memif_rdata_out(mem_rdata), .io_rdata_out(io_rdata_mem), .wb_reg_out(wb_reg_out_MEM), .wb_enable_out(wb_en_MEM), .wb_src_out(wb_src_out_MEM));

    // WB Stage instantiation
    WB_stage wb_stage(.clk(clk), .reset(reset), .df_wb_enable(df_wb_enable), .df_wb_reg(df_wb_reg), .df_wb_data(df_wb_data), .pc_in(pc_MEM), .iw_in(iw_MEM), .alu_in(alu_out_MEM), .memif_rdata_in(mem_rdata), .io_rdata_in(io_rdata_mem), .wb_reg_in(wb_reg_out_MEM), .wb_enable_in(wb_en_MEM), .wb_src_in(wb_src_out_MEM), .pc_out(pc_WB), .iw_out(iw_WB), .regif_wb_enable(wb_en_WB), .regif_wb_reg(wb_reg_out_WB), .regif_wb_data(wb_data));

    // Register File  instantiation
    rv32i_regs reg_file(.clk(clk), .reset(reset), .rs1_reg(rs1_reg), .rs2_reg(rs2_reg), .wb_enable(wb_en_WB), .wb_reg(wb_reg_out_WB), .wb_data(wb_data), .rs1_data(rs1_data_out), .rs2_data(rs2_data_out));

    // Dual Port RAM instantiation
    dual_port_ram_top dual_port_ram(.clk(clk), .i_addr(i_addr), .i_rdata(i_rdata), .d_addr(d_address), .d_rdata(d_rdata), .d_we(wb_enable), .d_be(d_be), .d_wdata(write_data));

    // I/O Module instantiation
    IO_module io(.clk(clk), .reset(reset), .io_addr(io_addr), .io_rdata(io_rdata), .io_we(io_we), .io_be(io_be), .io_wdata(io_wdata), .leds_out(leds_data));

    // ILA Instantiation
    ila_0 your_instance_name (
	.clk(clk), // input wire clk


	.probe0(pc_IF), // input wire [31:0]  probe0  
	.probe1(pc_ID), // input wire [31:0]  probe1 
	.probe2(pc_EX), // input wire [31:0]  probe2 
	.probe3(pc_MEM), // input wire [31:0]  probe3 
	.probe4(pc_WB), // input wire [31:0]  probe4 
	.probe5(iw_IF), // input wire [31:0]  probe5 
	.probe6(iw_ID), // input wire [31:0]  probe6 
	.probe7(iw_EX), // input wire [31:0]  probe7 
	.probe8(iw_MEM), // input wire [31:0]  probe8 
	.probe9(iw_WB), // input wire [31:0]  probe9
    .probe10(alu_out_EX), // input wire [31:0]  probe10
    .probe11(alu_out_MEM), // input wire [31:0]  probe11
    .probe12(wb_data), // input wire [31:0]  probe12
    .probe13(wb_reg_out_WB), // input wire [4:0]  probe13
    .probe14(wb_en_WB), // input wire [0:0]  probe14
    .probe15(ebreak), // input wire [0:0]  probe15
    .probe16(reset),
    .probe17(rs1_data_ID),
    .probe18(rs2_data_ID),
    .probe19(jump_enable),
    .probe20(jump_addr),
    .probe21(df_ex_data),
    .probe22(df_ex_reg),
    .probe23(df_ex_enable)
);

endmodule
