# PlayStation Emulation - CPU and Interrupt Controller Implementation

This directory contains an implementation of a PlayStation emulator's core components with a focus on the CPU and Interrupt Controller interaction.

## CPU and Interrupt Controller Interaction

### Purpose and Architecture

The PlayStation's interrupt system is a critical hardware component that enables asynchronous communication between peripherals and the CPU. This implementation centers around:

1. Direct hardware integration between the Interrupt Controller and CPU
2. Accurate replication of the physical interrupt signal pathways
3. Memory-mapped register access for software control

**Key Architectural Features:**
- The Interrupt Controller directly manipulates the CPU's CAUSE register
- PlayStation interrupt sources are mapped to specific CPU interrupt lines
- Memory-mapped registers (I_STAT, I_MASK) control interrupt behavior
- The CPU's Status Register (SR) controls global interrupt enabling/masking

### Runtime Interaction Examples

#### Example 1: VBLANK Interrupt Flow

When a vertical blanking interval occurs in the GPU:

1. GPU hardware signals a VBLANK interrupt to the Interrupt Controller
2. Interrupt Controller sets bit 0 in its internal status register (I_STAT)
3. If bit 0 in the mask register (I_MASK) is also set:
   - Interrupt Controller sets bit 8 in the CPU's CAUSE register
   - If the CPU's IE bit (SR.0) and IM0 bit (SR.8) are set, the CPU takes the interrupt
4. CPU jumps to the exception handler at 0x80000080 or 0xBFC00180
5. Software reads I_STAT (0x1F801070) to determine the interrupt source
6. Software acknowledges the interrupt by writing 1 to bit 0 of I_STAT
7. Interrupt Controller clears the corresponding bit in I_STAT and the CPU's CAUSE register

#### Example 2: Multiple Simultaneous Interrupts

When both a CD-ROM and a Controller interrupt occur:

1. Both peripherals signal their respective interrupts
2. Interrupt Controller sets bits 2 (CD-ROM) and 7 (Controller) in I_STAT
3. If both interrupts are unmasked in I_MASK:
   - CD-ROM interrupt sets bit 10 in the CPU's CAUSE register
   - Controller interrupt sets bit 10 in the CPU's CAUSE register (shared line)
4. CPU takes the interrupt if SR.IE and the appropriate IM bits are set
5. Software must read I_STAT to determine which specific peripheral triggered the interrupt
6. Software handles each interrupt and acknowledges them individually
7. Each acknowledgment clears its respective bit in I_STAT
8. When all interrupts on a shared line are cleared, the corresponding bit in CAUSE is cleared

#### Example 3: Interrupt Masking and Prioritization

When multiple high-priority tasks must be coordinated:

1. Software initializes by setting I_MASK to enable only critical interrupts (e.g., 0x0001 for VBLANK only)
2. During critical processing, software temporarily disables all interrupts by clearing SR.IE
3. For time-sensitive operations, software modifies I_MASK to allow specific interrupts
4. When a high-priority interrupt occurs, the interrupt handler can:
   - Save the current I_MASK value
   - Modify I_MASK to allow only specific nested interrupts
   - Re-enable interrupts (SR.IE) for nested handling
   - Restore the original I_MASK when finished

## Implementation Details

### CPU Core

The CPU core implements the MIPS R3000A architecture used in the PlayStation. Key features include:

- Full 32-bit MIPS R3000A instruction set implementation
- General purpose registers (32 x 32-bit)
- Coprocessor 0 (COP0) for system control and interrupt management
- Exception handling with priority for interrupts
- **Instruction and Data Caches** with proper cache control

### Interrupt Controller

The interrupt controller manages system interrupts and directly signals the CPU. Features include:

- Support for all PlayStation interrupt sources
- Hardware-based implementation that directly modifies CPU's CAUSE register
- Mapping of PlayStation interrupt types to appropriate CPU interrupt lines
- Memory-mapped I_STAT and I_MASK registers (0x1F801070 and 0x1F801074)

#### PlayStation to CPU Interrupt Mapping

| PlayStation Interrupt | I_STAT Bit | CPU Interrupt Line | CAUSE Register Bit |
|-----------------------|------------|-------------------|-------------------|
| VBLANK                | 0          | IP0               | 8                 |
| GPU                   | 1          | IP1               | 9                 |
| CDROM                 | 2          | IP2               | 10                |
| DMA                   | 3          | IP3               | 11                |
| TIMER0                | 4          | IP4               | 12                |
| TIMER1                | 5          | IP4               | 12                |
| TIMER2                | 6          | IP4               | 12                |
| Controller            | 7          | IP2               | 10                |
| SIO                   | 8          | IP2               | 10                |
| SPU                   | 9          | IP1               | 9                 |

## Testing

Comprehensive test cases demonstrate the CPU and Interrupt Controller interaction:

- `psx_emulation_interrupt_tests_005`: Tests the interrupt controller in isolation
- `psx_emulation_cpu_interrupt_tests_005`: Tests CPU response to interrupts
- `psx_emulation_hardware_interrupts_005`: Demonstrates complete interrupt flow scenarios

Run the tests with:

```
cd build
make
./bin/psx_emulation_hardware_interrupts_005
```

## Building

Use CMake to build the project:

```
mkdir build
cd build
cmake ..
make
``` 