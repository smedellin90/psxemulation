# PlayStation Emulation - CPU and Memory Implementation

This directory contains an implementation of a PlayStation emulator's core components:

1. CPU Core - MIPS R3000A compatible processor
2. Memory Controller - Manages system memory access
3. Interrupt Controller - Handles system interrupts

## CPU Core

The CPU core implements the MIPS R3000A architecture used in the PlayStation. Key features include:

- Full 32-bit MIPS R3000A instruction set implementation
- General purpose registers (32 x 32-bit)
- Coprocessor 0 (COP0) for system control
- Exception handling
- Branch delay slots
- HI/LO special registers for multiplication and division results
- **Instruction and Data Caches** with proper cache control

### Cache Implementation

The PlayStation's CPU includes two separate caches:

- **Instruction Cache (ICache)**: 4KB cache for instruction fetching
- **Data Cache (DCache)**: 1KB cache for data access

Both caches are implemented with the following characteristics:

- Direct-mapped organization
- 16-byte cache lines
- Write-through policy for data cache
- Support for cache isolation and swapping via COP0 Status Register
- Proper handling of uncached memory regions (KSEG1)
- Automatic loading of complete cache lines on miss

The cache implementation follows these key principles:

1. Proper separation between instruction fetches and data accesses
2. Cache coherency through write-through policy
3. Support for CP0 cache control operations
4. Tracking of cache statistics for performance analysis
5. Proper handling of cache line boundaries

#### Cache Control via CP0

The CPU implements the following cache control bits in the CP0 Status Register:

- **SR_ISC** (bit 16): Isolate Cache - When set, prevents cache from accessing main memory
- **SR_SWC** (bit 17): Swap Caches - When set, swaps the behavior of instruction and data caches
- **SR_CM** (bit 19): Cache Miss - Set when a cache miss occurs
- **SR_PE** (bit 20): Cache Parity Error - Set when a cache parity error occurs

### Memory Access

The CPU implements memory access through the following functions:

- `fetchInstruction()` - Uses the instruction cache for fetching instructions
- `readByte()`, `readHalf()`, `readWord()` - Use the data cache for reading memory
- `writeByte()`, `writeHalf()`, `writeWord()` - Use the data cache for writing memory
- Direct memory access functions for bypassing the cache

## Memory Controller

The memory controller handles access to the various memory regions in the PlayStation:

- 2MB Main RAM
- 512KB BIOS ROM
- 1KB Scratchpad RAM 
- Memory-mapped I/O for peripherals
- DMA capabilities for fast data transfer

The memory controller properly implements memory mapping, with these address ranges:

- 0x00000000-0x00200000: Main RAM
- 0x1F800000-0x1F800400: Scratchpad RAM
- 0x1F801000-0x1F802000: I/O Ports
- 0x1FC00000-0x1FC80000: BIOS ROM
- 0x80000000-0x9FFFFFFF: KSEG0 (Cached mirror of physical memory)
- 0xA0000000-0xBFFFFFFF: KSEG1 (Uncached mirror of physical memory)

## Interrupt Controller

The interrupt controller manages system interrupts and communicates with the CPU. Features include:

- Support for all PlayStation interrupt sources
- Interrupt masking
- Proper interrupt acknowledgment
- Integration with CPU's exception handling

## Testing

Comprehensive test cases are included for each component:

- CPU Tests: Verify instruction execution, exception handling, and cache behavior
- Memory Tests: Verify memory access, address translation, and DMA operations
- Interrupt Tests: Verify interrupt handling and prioritization

Run the tests with:

```
psx_emulation_cpu_tests_004
psx_emulation_memory_tests_004
psx_emulation_interrupt_tests_004
```

## Building

Use CMake to build the project:

```
mkdir build
cd build
cmake ..
make
``` 