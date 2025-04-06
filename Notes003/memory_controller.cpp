#include "memory_controller.h"
#include "interrupt_controller.h"
#include <fstream>
#include <iostream>
#include <cstring>

namespace PSX {

MemoryController::MemoryController(InterruptController* interruptController)
    : mainRam(2 * 1024 * 1024, 0)      // 2MB Main RAM
    , scratchpad(1024, 0)              // 1KB Scratchpad
    , biosRom(512 * 1024, 0)           // 512KB BIOS ROM
    , iCacheEnabled(false)
    , dCacheEnabled(false)
    , interruptController(interruptController)
{
}

void MemoryController::reset() {
    // Clear main RAM
    std::fill(mainRam.begin(), mainRam.end(), 0);
    
    // Clear scratchpad
    std::fill(scratchpad.begin(), scratchpad.end(), 0);
    
    // Note: BIOS ROM is not cleared on reset
    
    // Reset cache state
    iCacheEnabled = false;
    dCacheEnabled = false;
    
    // Clear I/O handlers (components should re-register after reset)
    ioHandlers.clear();
}

bool MemoryController::loadBios(const std::string& biosPath) {
    std::ifstream file(biosPath, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open BIOS file: " << biosPath << std::endl;
        return false;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    if (size > biosRom.size()) {
        std::cerr << "BIOS file too large: " << size << " bytes (max " << biosRom.size() << " bytes)" << std::endl;
        return false;
    }
    
    // Read the file
    if (!file.read(reinterpret_cast<char*>(biosRom.data()), size)) {
        std::cerr << "Failed to read BIOS file" << std::endl;
        return false;
    }
    
    std::cout << "Loaded BIOS: " << size << " bytes" << std::endl;
    return true;
}

uint32_t MemoryController::translateAddress(uint32_t address) const {
    // Convert virtual address to physical
    uint32_t physicalAddress = address;
    
    // Handle memory segments
    if ((address & 0xE0000000) == 0x80000000) {
        // KSEG0: Cached, subtract 0x80000000
        physicalAddress = address & 0x1FFFFFFF;
    } else if ((address & 0xE0000000) == 0xA0000000) {
        // KSEG1: Uncached, subtract 0xA0000000
        physicalAddress = address & 0x1FFFFFFF;
    } else if ((address & 0xC0000000) == 0x00000000) {
        // KUSEG: User space, may require TLB (not implemented here)
        physicalAddress = address & 0x1FFFFFFF;
    }
    // KSEG2 is typically not used in PlayStation
    
    return physicalAddress;
}

bool MemoryController::checkAlignment(uint32_t address, uint32_t size) const {
    // Check if the address is properly aligned for the access size
    return (size == 1) || (size == 2 && (address % 2 == 0)) || (size == 4 && (address % 4 == 0));
}

uint8_t MemoryController::readByte(uint32_t address) {
    uint32_t physAddr = translateAddress(address);
    
    // Main RAM: 0x00000000-0x00200000
    if (physAddr < 0x00200000) {
        return mainRam[physAddr];
    }
    
    // Scratchpad: 0x1F800000-0x1F800400
    else if (physAddr >= 0x1F800000 && physAddr < 0x1F800400) {
        return scratchpad[physAddr - 0x1F800000];
    }
    
    // I/O Ports: 0x1F801000-0x1F802000
    else if (physAddr >= 0x1F801000 && physAddr < 0x1F802000) {
        // Check if there's a registered handler
        for (const auto& handler : ioHandlers) {
            if (physAddr >= handler.baseAddress && 
                physAddr < handler.baseAddress + handler.size) {
                if (handler.readCB) {
                    // Most I/O ports are 32-bit, so we need to read a word and extract the byte
                    uint32_t aligned = physAddr & ~0x3;
                    uint32_t value = handler.readCB(aligned);
                    int shift = (physAddr & 0x3) * 8;
                    return (value >> shift) & 0xFF;
                }
                break;
            }
        }
        
        std::cerr << "Unhandled I/O read byte: 0x" << std::hex << physAddr << std::endl;
        return 0xFF;
    }
    
    // BIOS ROM: 0x1FC00000-0x1FC80000
    else if (physAddr >= 0x1FC00000 && physAddr < 0x1FC80000) {
        return biosRom[physAddr - 0x1FC00000];
    }
    
    // Unhandled memory region
    std::cerr << "Unhandled memory read byte: 0x" << std::hex << physAddr << std::endl;
    return 0xFF;
}

uint16_t MemoryController::readHalf(uint32_t address) {
    // Check alignment
    if (!checkAlignment(address, 2)) {
        std::cerr << "Unaligned half-word read: 0x" << std::hex << address << std::endl;
        // In real hardware this would trigger an exception
        return 0xFFFF;
    }
    
    uint32_t physAddr = translateAddress(address);
    
    // Main RAM: 0x00000000-0x00200000
    if (physAddr < 0x00200000) {
        return *reinterpret_cast<uint16_t*>(&mainRam[physAddr]);
    }
    
    // Scratchpad: 0x1F800000-0x1F800400
    else if (physAddr >= 0x1F800000 && physAddr < 0x1F800400) {
        return *reinterpret_cast<uint16_t*>(&scratchpad[physAddr - 0x1F800000]);
    }
    
    // I/O Ports: 0x1F801000-0x1F802000
    else if (physAddr >= 0x1F801000 && physAddr < 0x1F802000) {
        // Check if there's a registered handler
        for (const auto& handler : ioHandlers) {
            if (physAddr >= handler.baseAddress && 
                physAddr < handler.baseAddress + handler.size) {
                if (handler.readCB) {
                    // Most I/O ports are 32-bit, so we need to read a word and extract the half-word
                    uint32_t aligned = physAddr & ~0x3;
                    uint32_t value = handler.readCB(aligned);
                    int shift = (physAddr & 0x2) * 8;
                    return (value >> shift) & 0xFFFF;
                }
                break;
            }
        }
        
        std::cerr << "Unhandled I/O read half: 0x" << std::hex << physAddr << std::endl;
        return 0xFFFF;
    }
    
    // BIOS ROM: 0x1FC00000-0x1FC80000
    else if (physAddr >= 0x1FC00000 && physAddr < 0x1FC80000) {
        return *reinterpret_cast<uint16_t*>(&biosRom[physAddr - 0x1FC00000]);
    }
    
    // Unhandled memory region
    std::cerr << "Unhandled memory read half: 0x" << std::hex << physAddr << std::endl;
    return 0xFFFF;
}

uint32_t MemoryController::readWord(uint32_t address) {
    // Check alignment
    if (!checkAlignment(address, 4)) {
        std::cerr << "Unaligned word read: 0x" << std::hex << address << std::endl;
        // In real hardware this would trigger an exception
        return 0xFFFFFFFF;
    }
    
    uint32_t physAddr = translateAddress(address);
    
    // Main RAM: 0x00000000-0x00200000
    if (physAddr < 0x00200000) {
        return *reinterpret_cast<uint32_t*>(&mainRam[physAddr]);
    }
    
    // Scratchpad: 0x1F800000-0x1F800400
    else if (physAddr >= 0x1F800000 && physAddr < 0x1F800400) {
        return *reinterpret_cast<uint32_t*>(&scratchpad[physAddr - 0x1F800000]);
    }
    
    // I/O Ports: 0x1F801000-0x1F802000
    else if (physAddr >= 0x1F801000 && physAddr < 0x1F802000) {
        // Check if there's a registered handler
        for (const auto& handler : ioHandlers) {
            if (physAddr >= handler.baseAddress && 
                physAddr < handler.baseAddress + handler.size) {
                if (handler.readCB) {
                    return handler.readCB(physAddr);
                }
                break;
            }
        }
        
        std::cerr << "Unhandled I/O read word: 0x" << std::hex << physAddr << std::endl;
        return 0xFFFFFFFF;
    }
    
    // BIOS ROM: 0x1FC00000-0x1FC80000
    else if (physAddr >= 0x1FC00000 && physAddr < 0x1FC80000) {
        return *reinterpret_cast<uint32_t*>(&biosRom[physAddr - 0x1FC00000]);
    }
    
    // Handle addresses beyond the end of main RAM but still within the addressable physical memory
    // This is needed for proper mirroring in the memory mapping tests
    else if (physAddr <= 0x007FFFFF) {  // We'll limit to 8MB for reasonable memory mirroring
        // Wrap around within the 2MB RAM, treating it as if addresses repeat
        uint32_t wrappedAddr = physAddr & 0x001FFFFF;  // Mask to 2MB
        return *reinterpret_cast<uint32_t*>(&mainRam[wrappedAddr]);
    }
    
    // Unhandled memory region
    std::cerr << "Unhandled memory read word: 0x" << std::hex << physAddr << std::endl;
    return 0xFFFFFFFF;
}

void MemoryController::writeByte(uint32_t address, uint8_t value) {
    uint32_t physAddr = translateAddress(address);
    
    // Main RAM: 0x00000000-0x00200000
    if (physAddr < 0x00200000) {
        mainRam[physAddr] = value;
        return;
    }
    
    // Scratchpad: 0x1F800000-0x1F800400
    else if (physAddr >= 0x1F800000 && physAddr < 0x1F800400) {
        scratchpad[physAddr - 0x1F800000] = value;
        return;
    }
    
    // I/O Ports: 0x1F801000-0x1F802000
    else if (physAddr >= 0x1F801000 && physAddr < 0x1F802000) {
        // Check if there's a registered handler
        for (const auto& handler : ioHandlers) {
            if (physAddr >= handler.baseAddress && 
                physAddr < handler.baseAddress + handler.size) {
                if (handler.writeCB) {
                    // Most I/O ports require full 32-bit writes, so we need to read-modify-write
                    uint32_t aligned = physAddr & ~0x3;
                    uint32_t oldValue = 0;
                    for (const auto& readHandler : ioHandlers) {
                        if (aligned >= readHandler.baseAddress && 
                            aligned < readHandler.baseAddress + readHandler.size) {
                            if (readHandler.readCB) {
                                oldValue = readHandler.readCB(aligned);
                                break;
                            }
                        }
                    }
                    
                    int shift = (physAddr & 0x3) * 8;
                    uint32_t mask = ~(0xFF << shift);
                    uint32_t newValue = (oldValue & mask) | (value << shift);
                    handler.writeCB(aligned, newValue);
                    return;
                }
                break;
            }
        }
        
        std::cerr << "Unhandled I/O write byte: 0x" << std::hex << physAddr << ", value: 0x" << std::hex << (int)value << std::endl;
        return;
    }
    
    // BIOS ROM: 0x1FC00000-0x1FC80000 (read-only)
    else if (physAddr >= 0x1FC00000 && physAddr < 0x1FC80000) {
        std::cerr << "Attempted write to BIOS ROM: 0x" << std::hex << physAddr << std::endl;
        return;
    }
    
    // Handle addresses beyond the end of main RAM but still within the addressable physical memory
    else if (physAddr <= 0x007FFFFF) {  // We'll limit to 8MB for reasonable memory mirroring
        // Wrap around within the 2MB RAM, treating it as if addresses repeat
        uint32_t wrappedAddr = physAddr & 0x001FFFFF;  // Mask to 2MB
        mainRam[wrappedAddr] = value;
        return;
    }
    
    // Unhandled memory region
    std::cerr << "Unhandled memory write byte: 0x" << std::hex << physAddr << ", value: 0x" << std::hex << (int)value << std::endl;
}

void MemoryController::writeHalf(uint32_t address, uint16_t value) {
    // Check alignment
    if (!checkAlignment(address, 2)) {
        std::cerr << "Unaligned half-word write: 0x" << std::hex << address << std::endl;
        // In real hardware this would trigger an exception
        return;
    }
    
    uint32_t physAddr = translateAddress(address);
    
    // Main RAM: 0x00000000-0x00200000
    if (physAddr < 0x00200000) {
        *reinterpret_cast<uint16_t*>(&mainRam[physAddr]) = value;
        return;
    }
    
    // Scratchpad: 0x1F800000-0x1F800400
    else if (physAddr >= 0x1F800000 && physAddr < 0x1F800400) {
        *reinterpret_cast<uint16_t*>(&scratchpad[physAddr - 0x1F800000]) = value;
        return;
    }
    
    // I/O Ports: 0x1F801000-0x1F802000
    else if (physAddr >= 0x1F801000 && physAddr < 0x1F802000) {
        // Check if there's a registered handler
        for (const auto& handler : ioHandlers) {
            if (physAddr >= handler.baseAddress && 
                physAddr < handler.baseAddress + handler.size) {
                if (handler.writeCB) {
                    // Most I/O ports require full 32-bit writes, so we need to read-modify-write
                    uint32_t aligned = physAddr & ~0x3;
                    uint32_t oldValue = 0;
                    for (const auto& readHandler : ioHandlers) {
                        if (aligned >= readHandler.baseAddress && 
                            aligned < readHandler.baseAddress + readHandler.size) {
                            if (readHandler.readCB) {
                                oldValue = readHandler.readCB(aligned);
                                break;
                            }
                        }
                    }
                    
                    int shift = (physAddr & 0x2) * 8;
                    uint32_t mask = ~(0xFFFF << shift);
                    uint32_t newValue = (oldValue & mask) | (value << shift);
                    handler.writeCB(aligned, newValue);
                    return;
                }
                break;
            }
        }
        
        std::cerr << "Unhandled I/O write half: 0x" << std::hex << physAddr << ", value: 0x" << std::hex << value << std::endl;
        return;
    }
    
    // BIOS ROM: 0x1FC00000-0x1FC80000 (read-only)
    else if (physAddr >= 0x1FC00000 && physAddr < 0x1FC80000) {
        std::cerr << "Attempted write to BIOS ROM: 0x" << std::hex << physAddr << std::endl;
        return;
    }
    
    // Handle addresses beyond the end of main RAM but still within the addressable physical memory
    else if (physAddr <= 0x007FFFFF) {  // We'll limit to 8MB for reasonable memory mirroring
        // Wrap around within the 2MB RAM, treating it as if addresses repeat
        uint32_t wrappedAddr = physAddr & 0x001FFFFF;  // Mask to 2MB
        *reinterpret_cast<uint16_t*>(&mainRam[wrappedAddr]) = value;
        return;
    }
    
    // Unhandled memory region
    std::cerr << "Unhandled memory write half: 0x" << std::hex << physAddr << ", value: 0x" << std::hex << value << std::endl;
}

void MemoryController::writeWord(uint32_t address, uint32_t value) {
    // Check alignment
    if (!checkAlignment(address, 4)) {
        std::cerr << "Unaligned word write: 0x" << std::hex << address << std::endl;
        // In real hardware this would trigger an exception
        return;
    }
    
    uint32_t physAddr = translateAddress(address);
    
    // Main RAM: 0x00000000-0x00200000
    if (physAddr < 0x00200000) {
        *reinterpret_cast<uint32_t*>(&mainRam[physAddr]) = value;
        return;
    }
    
    // Scratchpad: 0x1F800000-0x1F800400
    else if (physAddr >= 0x1F800000 && physAddr < 0x1F800400) {
        *reinterpret_cast<uint32_t*>(&scratchpad[physAddr - 0x1F800000]) = value;
        return;
    }
    
    // I/O Ports: 0x1F801000-0x1F802000
    else if (physAddr >= 0x1F801000 && physAddr < 0x1F802000) {
        // Check if there's a registered handler
        for (const auto& handler : ioHandlers) {
            if (physAddr >= handler.baseAddress && 
                physAddr < handler.baseAddress + handler.size) {
                if (handler.writeCB) {
                    handler.writeCB(physAddr, value);
                    return;
                }
                break;
            }
        }
        
        std::cerr << "Unhandled I/O write word: 0x" << std::hex << physAddr << ", value: 0x" << std::hex << value << std::endl;
        return;
    }
    
    // BIOS ROM: 0x1FC00000-0x1FC80000 (read-only)
    else if (physAddr >= 0x1FC00000 && physAddr < 0x1FC80000) {
        std::cerr << "Attempted write to BIOS ROM: 0x" << std::hex << physAddr << std::endl;
        return;
    }
    
    // Handle addresses beyond the end of main RAM but still within the addressable physical memory
    else if (physAddr <= 0x007FFFFF) {  // We'll limit to 8MB for reasonable memory mirroring
        // Wrap around within the 2MB RAM, treating it as if addresses repeat
        uint32_t wrappedAddr = physAddr & 0x001FFFFF;  // Mask to 2MB
        *reinterpret_cast<uint32_t*>(&mainRam[wrappedAddr]) = value;
        return;
    }
    
    // Unhandled memory region
    std::cerr << "Unhandled memory write word: 0x" << std::hex << physAddr << ", value: 0x" << std::hex << value << std::endl;
}

void MemoryController::registerIOReadHandler(uint32_t baseAddress, uint32_t size, ReadCallback callback) {
    // Ensure the address is in the I/O range
    uint32_t physAddr = translateAddress(baseAddress);
    if (physAddr < 0x1F801000 || physAddr >= 0x1F802000) {
        std::cerr << "Attempted to register I/O handler outside I/O range: 0x" << std::hex << physAddr << std::endl;
        return;
    }
    
    // Find if there's an existing handler for this range
    for (auto& handler : ioHandlers) {
        if (physAddr >= handler.baseAddress && 
            physAddr < handler.baseAddress + handler.size) {
            // Update the existing handler
            handler.readCB = callback;
            return;
        }
    }
    
    // Add a new handler
    ioHandlers.push_back({
        .baseAddress = physAddr,
        .size = size,
        .readCB = callback,
        .writeCB = nullptr,
    });
}

void MemoryController::registerIOWriteHandler(uint32_t baseAddress, uint32_t size, WriteCallback callback) {
    // Ensure the address is in the I/O range
    uint32_t physAddr = translateAddress(baseAddress);
    if (physAddr < 0x1F801000 || physAddr >= 0x1F802000) {
        std::cerr << "Attempted to register I/O handler outside I/O range: 0x" << std::hex << physAddr << std::endl;
        return;
    }
    
    // Find if there's an existing handler for this range
    for (auto& handler : ioHandlers) {
        if (physAddr >= handler.baseAddress && 
            physAddr < handler.baseAddress + handler.size) {
            // Update the existing handler
            handler.writeCB = callback;
            return;
        }
    }
    
    // Add a new handler
    IOHandler handler;
    handler.baseAddress = physAddr;
    handler.size = size;
    handler.readCB = nullptr; // Read callback not set
    handler.writeCB = callback;
    ioHandlers.push_back(handler);
}

void MemoryController::dmaTransfer(uint32_t srcAddress, uint32_t destAddress, uint32_t size) {
    // Basic DMA transfer implementation
    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t data = readWord(srcAddress + i);
        writeWord(destAddress + i, data);
    }
}

void MemoryController::invalidateICache(uint32_t address, uint32_t size) {
    // Simple cache invalidation - in a real emulator, this would invalidate 
    // specific cache lines. Here we just note that cache would be affected.
    std::cout << "Invalidate ICache: 0x" << std::hex << address << ", size: " << size << std::endl;
}

void MemoryController::invalidateDCache(uint32_t address, uint32_t size) {
    // Simple cache invalidation - in a real emulator, this would invalidate 
    // specific cache lines. Here we just note that cache would be affected.
    std::cout << "Invalidate DCache: 0x" << std::hex << address << ", size: " << size << std::endl;
}

} // namespace PSX 