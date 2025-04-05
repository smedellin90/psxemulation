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

// Forward declarations
namespace PSXTest {
    void runTests();
    void runInterruptControllerTests();
}

int main() {
    std::cout << "Running PlayStation CPU Emulator Tests\n";
    PSXTest::runTests();
    
    std::cout << "\n\nRunning PlayStation Interrupt Controller Tests\n";
    PSXTest::runInterruptControllerTests();
    
    return 0;
}