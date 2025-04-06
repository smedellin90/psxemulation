#pragma once

#include <cstdint>
#include <functional>
#include <array>
#include <vector>

namespace PSX {

// Forward declarations
class InterruptController;

/**
 * PlayStation Memory Map:
 * 
 * 0x00000000-0x00200000   2MB   Main RAM (KUSEG)
 * 0x1F000000-0x1F800000   8MB   Expansion Region 1
 * 0x1F800000-0x1F800400   1KB   Scratchpad RAM
 * 0x1F801000-0x1F802000   4KB   I/O Ports
 * 0x1F802000-0x1F804000   8KB   Expansion Region 2
 * 0x1FC00000-0x1FC80000 512KB   BIOS ROM
 * 0x80000000+             --    KSEG0 (Mirror of physical memory, cached)
 * 0xA0000000+             --    KSEG1 (Mirror of physical memory, uncached)
 * 0xC0000000+             --    KSEG2 (Usually unused)
 */
class MemoryController {
public:
    // Memory access callback types (for hardware components)
    using ReadCallback = std::function<uint32_t(uint32_t)>;
    using WriteCallback = std::function<void(uint32_t, uint32_t)>;

    /**
     * Constructor.
     * 
     * @param interruptController Pointer to the interrupt controller.
     */
    MemoryController(InterruptController* interruptController);

    /**
     * Reset the memory controller and all memory contents.
     */
    void reset();

    /**
     * Load the BIOS ROM from a file.
     * 
     * @param biosPath Path to the BIOS ROM file.
     * @return True if successful, false otherwise.
     */
    bool loadBios(const std::string& biosPath);

    /**
     * Memory read methods
     */
    uint8_t readByte(uint32_t address);
    uint16_t readHalf(uint32_t address);
    uint32_t readWord(uint32_t address);

    /**
     * Memory write methods
     */
    void writeByte(uint32_t address, uint8_t value);
    void writeHalf(uint32_t address, uint16_t value);
    void writeWord(uint32_t address, uint32_t value);

    /**
     * Register I/O handlers for specific address ranges.
     */
    void registerIOReadHandler(uint32_t baseAddress, uint32_t size, ReadCallback callback);
    void registerIOWriteHandler(uint32_t baseAddress, uint32_t size, WriteCallback callback);

    /**
     * Direct memory access functions (for DMA controller)
     */
    void dmaTransfer(uint32_t srcAddress, uint32_t destAddress, uint32_t size);

    /**
     * Cache control functions
     */
    void invalidateICache(uint32_t address, uint32_t size);
    void invalidateDCache(uint32_t address, uint32_t size);

private:
    // Translate virtual address to physical address
    uint32_t translateAddress(uint32_t address) const;

    // Check if address is valid for given access type and size
    bool checkAlignment(uint32_t address, uint32_t size) const;

    // Memory regions
    std::vector<uint8_t> mainRam;      // 2MB Main RAM
    std::vector<uint8_t> scratchpad;   // 1KB Scratchpad RAM
    std::vector<uint8_t> biosRom;      // 512KB BIOS ROM

    // I/O port handlers
    struct IOHandler {
        uint32_t baseAddress;
        uint32_t size;
        ReadCallback readCB;
        WriteCallback writeCB;
    };

    std::vector<IOHandler> ioHandlers;

    // Cache state (TLB would be here in a more complete implementation)
    bool iCacheEnabled;
    bool dCacheEnabled;

    // Reference to interrupt controller (for reporting bus errors)
    InterruptController* interruptController;

    // Access timing constants (in cycles)
    static constexpr int RAM_READ_CYCLES = 5;     // Main RAM
    static constexpr int RAM_WRITE_CYCLES = 5;    // Main RAM
    static constexpr int ROM_ACCESS_CYCLES = 8;   // BIOS ROM
    static constexpr int CACHE_ACCESS_CYCLES = 1; // Cached access
    static constexpr int IO_ACCESS_CYCLES = 10;   // I/O ports (varies in reality)
};

} // namespace PSX 