#include "cpu_core.h"
#include <iostream>
#include <map>

// Simple memory simulation for testing
class Memory {
private:
    std::map<uint32_t, uint32_t> contents;

public:
    uint32_t read(uint32_t address) {
        // Align to word boundary
        address &= ~0x3;
        
        auto it = contents.find(address);
        if (it != contents.end()) {
            return it->second;
        }

        // Default value for uninitialized memory
        return 0xDEADBEEF;  // Easily recognizable pattern
    }

    void write(uint32_t address, uint32_t value) {
        // Align to word boundary
        address &= ~0x3;
        contents[address] = value;
    }
};

int main() {
    // Create CPU instance
    PSX::CPU cpu;
    
    // Create memory simulation
    Memory mem;
    
    // Set up memory callbacks
    cpu.setMemoryCallbacks(
        [&mem](uint32_t address) { return mem.read(address); },
        [&mem](uint32_t address, uint32_t value) { mem.write(address, value); }
    );
    
    // Initialize CPU
    cpu.reset();
    
    // Add a simple test instruction at the BIOS entry point (just for testing)
    // This is an ADDIU instruction: addiu $t0, $zero, 42
    mem.write(0xBFC00000, 0x24080042);
    
    // Execute one instruction and dump state
    std::cout << "Executing instruction..." << std::endl;
    cpu.executeInstruction();
    
    // Print register state
    cpu.dumpState();
    
    // Check if the instruction worked
    std::cout << "Register T0 value: " << cpu.getRegister(PSX::CPU::T0) << std::endl;
    
    return 0;
}