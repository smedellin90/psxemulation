### CPU Architecture
- Complete register set implementation (32 general-purpose registers)
- Coprocessor 0 (system control) registers
- Program counter and delay slot handling
- HI/LO registers for multiplication/division results

### Instruction Sets
- ALU operations (ADD, SUB, AND, OR, XOR, etc.)
- Branch instructions (BEQ, BNE, BGTZ, BLEZ)
- Jump instructions (J, JAL, JR, JALR)
- Load/Store operations (LW, LB, SW, SB, etc.) // TODO Unaligned Load/Store Operations.
- Multiplication/Division operations with dedicated HI/LO registers (MULT, MULTU, DIV, DIVU, MFHI, MFLO, MTHI, MTLO)
- Bit manipulation (shifts, logical operations)
- Special instruction handling (SPECIAL, REGIMM opcodes)

### System Features
- Exception handling (address errors, reserved instructions, etc.)
- Interrupt handling system
- Memory read/write callbacks for interfacing with system memory
- Debug utilities (register dumps, state display)

### Memory Access
- Properly aligned and unaligned access handling
- Byte, halfword, and word read/write operations
- Exception triggering for misaligned accesses

### Coprocessors
- Basic Coprocessor 0 (system control) support

