# RISC-V Core (Verilog Implementation)

This project implements a **single-cycle RV32I subset RISC-V processor** in pure **Verilog**.  
It supports a small instruction set (ADDI, ADD, SUB, AND, OR, XOR, LW, SW, BEQ, BNE, JAL, JALR).  
The implementation is aimed at **simulation and verification** (not FPGA-synth optimized).

---

## Files

1. **riscv_core.v**  
   - Implements the RISC-V core (PC, instruction decode, ALU, register file, data memory).  
   - Contains runtime safety checks (x0 immutability, PC alignment).

2. **tb_riscv.v**  
   - Verilog testbench with clock/reset, waveform dump (`riscv.vcd`).  
   - Loads program from `instr.mem`.  
   - Includes simple self-checking (`check_reg` task).

3. **instr.mem**  
   - Program in hex (RISC-V machine code).  
   - Contains a small program that tests arithmetic, load/store, branch, and jump.

---

## Example Program (`instr.mem`)

```
addi x1, x0, 5      # x1 = 5
addi x2, x0, 10     # x2 = 10
add  x3, x1, x2     # x3 = 15
sw   x3, 0(x0)      # store 15 into dmem[0]
lw   x4, 0(x0)      # load 15 into x4
beq  x4, x3, +8     # branch taken, skip addi x5
addi x5, x0, 1      # skipped
jal  x6, 8          # jump to xor instruction
xor  x7, x1, x2     # x7 = 15
```

Expected results after simulation:  
- `x1 = 5, x2 = 10, x3 = 15, x4 = 15, x5 = 0, x6 = 32, x7 = 15`  
- `dmem[0] = 15`  

---

## How to Run

### Using Icarus Verilog
```bash
iverilog -o simv riscv_core.v tb_riscv.v
vvp simv
gtkwave riscv.vcd
```

### Using ModelSim / Questa
```tcl
vlog riscv_core.v tb_riscv.v
vsim tb_riscv
run -all
```

---

## Notes

- Designed for **learning / verification** (not pipelined, no hazards).  
- Register x0 is hard-wired to 0.  
- Branches and jumps are resolved in single-cycle.  
- Easily extensible to more instructions.

---

## License
Free to use for academic / learning purposes.
