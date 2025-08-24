# RV32I RISC-V Processor (5-Stage Pipeline)

This repository contains the hardware implementation of a 5-stage pipelined 32-bit RISC-V processor supporting the full RV32I + ZM\* ISA extensions. The design includes pipeline hazard detection and resolution, data forwarding, and dual-port memory, built and tested on the BLACKBOARD FPGA platform using Vivado.

## Features

* 5-Stage Pipelined Processor
  IF → ID → EX → MEM → WB stages implemented with complete control and data path logic.

* ISA Support
  Implements the full RV32I instruction set with ZMul extension for hardware multiplication.

* Pipeline Hazard Detection & Resolution

  * Data hazards: forwarding and stalling mechanisms
  * Control hazards: branch handling and pipeline flushes

* True Dual-Port Memory
  Simultaneous instruction and data memory access for improved throughput.

* FPGA Validation
  Built for the BLACKBOARD FPGA with on-chip debugging using Vivado ILA.

## Tools & Technologies

* Hardware Description Language: SystemVerilog
* FPGA Toolchain: Xilinx Vivado
* Target Platform: BLACKBOARD FPGA
* Debugging: Integrated Logic Analyzer (ILA)

```

## FPGA & ILA Verification

Cycle-accurate behavior and internal signals were captured using Vivado ILA, ensuring correct pipeline operation across all test scenarios.

## Getting Started

1. Clone the repository:

   ```bash
   git clone git@github.com:Yasasvi9/RV32I-RISCV-Processor.git
   ```
2. Open the project in Xilinx Vivado.
3. Synthesize, implement, and program the FPGA board.
4. Use the VIVADO ILA for runtime debugging.

## License

This project is licensed under a proprietary license.

## Acknowledgments

* RISC-V International for the open-source ISA
* Xilinx for Vivado Design Suite
* University labs for providing the BLACKBOARD FPGA platform
