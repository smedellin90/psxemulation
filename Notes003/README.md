# PlayStation Memory Controller Implementation

This directory contains the implementation of a PlayStation memory controller, which is responsible for managing all memory access within the emulator. This includes:

- Handling the PlayStation's memory map
- Translating virtual addresses to physical addresses
- Managing access to different memory regions (RAM, ROM, I/O)
- Providing memory-mapped I/O for other hardware components
- Basic DMA functionality
- Cache control support

## Memory Map

The PlayStation uses the following memory map:

| Address Range           | Size    | Description                                   |
|-------------------------|---------|-----------------------------------------------|
| 0x00000000-0x00200000   | 2MB     | Main RAM (KUSEG)                             |
| 0x1F000000-0x1F800000   | 8MB     | Expansion Region 1                           |
| 0x1F800000-0x1F800400   | 1KB     | Scratchpad RAM                               |
| 0x1F801000-0x1F802000   | 4KB     | I/O Ports                                    |
| 0x1F802000-0x1F804000   | 8KB     | Expansion Region 2                           |
| 0x1FC00000-0x1FC80000   | 512KB   | BIOS ROM                                     |
| 0x80000000+             | --      | KSEG0 (Mirror of physical memory, cached)    |
| 0xA0000000+             | --      | KSEG1 (Mirror of physical memory, uncached)  |
| 0xC0000000+             | --      | KSEG2 (Usually unused)                       |

## Key Features

### Address Translation

The memory controller handles the translation of virtual addresses to physical addresses according to the MIPS architecture's memory segments:

- **KUSEG** (0x00000000-0x7FFFFFFF): User space, mapped to physical memory at 0x00000000-0x1FFFFFFF
- **KSEG0** (0x80000000-0x9FFFFFFF): Kernel space, cached, mapped to physical memory at 0x00000000-0x1FFFFFFF
- **KSEG1** (0xA0000000-0xBFFFFFFF): Kernel space, uncached, mapped to physical memory at 0x00000000-0x1FFFFFFF
- **KSEG2** (0xC0000000-0xFFFFFFFF): Additional kernel space, typically not used in the PlayStation

### Memory Regions

The implementation provides access to the following memory regions:

- **Main RAM** (2MB): General-purpose memory for the system
- **Scratchpad RAM** (1KB): Fast, on-chip memory for temporary data
- **BIOS ROM** (512KB): Read-only memory containing the PlayStation BIOS
- **I/O Ports**: Memory-mapped I/O for hardware components

### I/O Port Handlers

The memory controller implements a callback system that allows hardware components to register handlers for memory-mapped I/O regions. This enables:

- Registering read handlers for I/O ports
- Registering write handlers for I/O ports
- Proper byte, half-word, and word access to I/O devices

### Error Handling

The implementation includes error handling for:

- Unaligned memory access
- Writes to read-only memory (BIOS ROM)
- Access to unhandled memory regions

### DMA Support

The memory controller provides a basic DMA (Direct Memory Access) interface that allows for efficient data transfer between memory regions without CPU intervention.

## Usage

### Basic Memory Access

```cpp
// Create the memory controller
PSX::InterruptController ic;
PSX::MemoryController mem(&ic);

// Memory read operations
uint8_t byte = mem.readByte(0x00000100);
uint16_t half = mem.readHalf(0x00000102);
uint32_t word = mem.readWord(0x00000104);

// Memory write operations
mem.writeByte(0x00000100, 0x12);
mem.writeHalf(0x00000102, 0x3456);
mem.writeWord(0x00000104, 0x789ABCDE);
```

### Registering I/O Handlers

```cpp
// Create a callback for reading from an I/O port
auto readHandler = [](uint32_t address) -> uint32_t {
    // Handle the read operation
    return someRegisterValue;
};

// Register the read handler
mem.registerIOReadHandler(0x1F801000, 4, readHandler);

// Create a callback for writing to an I/O port
auto writeHandler = [](uint32_t address, uint32_t value) {
    // Handle the write operation
    someRegisterValue = value;
};

// Register the write handler
mem.registerIOWriteHandler(0x1F801000, 4, writeHandler);
```

### DMA Transfer

```cpp
// Transfer 64 bytes from address 0x00000000 to 0x00100000
mem.dmaTransfer(0x00000000, 0x00100000, 64);
```

## Future Enhancements

Future improvements to the memory controller could include:

1. Implementing a proper cache emulation (ICache and DCache)
2. More accurate timing simulation for memory access
3. Full implementation of the PlayStation's DMA controller
4. TLB (Translation Lookaside Buffer) emulation if needed
5. Support for expansion memory

## Tests

The implementation includes comprehensive tests covering:

1. Basic initialization
2. RAM and ROM access
3. Scratchpad RAM access
4. Memory-mapped I/O
5. Address translation
6. DMA transfers
7. Error handling

Run the tests with:

```
psx_emulation_memory_tests
``` 