#include "memory_controller.h"
#include "interrupt_controller.h"
#include <iostream>
#include <cassert>
#include <memory>
#include <string>
#include <functional>
#include <vector>

namespace PSXTest {

// Simple test framework
class TestRunner {
private:
    int passedTests = 0;
    int totalTests = 0;
    std::string currentTestName;

public:
    void check(bool condition, const std::string& message) {
        if (condition) {
            std::cout << "  ✓ " << message << std::endl;
            passedTests++;
        } else {
            std::cout << "  ✗ " << message << std::endl;
        }
        totalTests++;
    }
    
    template<typename T>
    void checkEqual(const T& actual, const T& expected, const std::string& message) {
        if (actual == expected) {
            std::cout << "  ✓ " << message << " (Got: " << actual << ")" << std::endl;
            passedTests++;
        } else {
            std::cout << "  ✗ " << message << " (Expected: " << expected << ", Got: " << actual << ")" << std::endl;
        }
        totalTests++;
    }
    
    void addTest(const std::string& name, std::function<void()> testFunc) {
        std::cout << "Running test: " << name << std::endl;
        currentTestName = name;
        testFunc();
        std::cout << std::endl;
    }
    
    void runAll() {
        std::cout << "===========================================" << std::endl;
        std::cout << "Test Results: " << passedTests << " passed, " 
                  << (totalTests - passedTests) << " failed, "
                  << totalTests << " total" << std::endl;
        std::cout << "===========================================" << std::endl;
    }
};

// Run the memory controller tests
void runMemoryControllerTests() {
    TestRunner runner;
    
    // Test 1: Basic Initialization
    runner.addTest("Memory Controller Initialization", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Test basic reads from ROM (should be 0 initially)
        runner.checkEqual<uint8_t>(mem.readByte(0x1FC00000), 0, "BIOS ROM byte read should be 0 initially");
        runner.checkEqual<uint16_t>(mem.readHalf(0x1FC00000), 0, "BIOS ROM half read should be 0 initially");
        runner.checkEqual<uint32_t>(mem.readWord(0x1FC00000), 0, "BIOS ROM word read should be 0 initially");
        
        // Test basic reads from RAM (should be 0 initially)
        runner.checkEqual<uint8_t>(mem.readByte(0x00000000), 0, "Main RAM byte read should be 0 initially");
        runner.checkEqual<uint16_t>(mem.readHalf(0x00000000), 0, "Main RAM half read should be 0 initially");
        runner.checkEqual<uint32_t>(mem.readWord(0x00000000), 0, "Main RAM word read should be 0 initially");
        
        // Test basic reads from scratchpad (should be 0 initially)
        runner.checkEqual<uint8_t>(mem.readByte(0x1F800000), 0, "Scratchpad byte read should be 0 initially");
        runner.checkEqual<uint16_t>(mem.readHalf(0x1F800000), 0, "Scratchpad half read should be 0 initially");
        runner.checkEqual<uint32_t>(mem.readWord(0x1F800000), 0, "Scratchpad word read should be 0 initially");
    });
    
    // Test 2: Basic RAM Access
    runner.addTest("Main RAM Access", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Write and read a byte
        mem.writeByte(0x00000000, 0x12);
        runner.checkEqual<uint8_t>(mem.readByte(0x00000000), 0x12, "Byte write/read");
        
        // Write and read a half-word
        mem.writeHalf(0x00000002, 0x3456);
        runner.checkEqual<uint16_t>(mem.readHalf(0x00000002), 0x3456, "Half-word write/read");
        
        // Write and read a word
        mem.writeWord(0x00000004, 0x789ABCDE);
        runner.checkEqual<uint32_t>(mem.readWord(0x00000004), 0x789ABCDE, "Word write/read");
        
        // Check byte access to word data
        runner.checkEqual<uint8_t>(mem.readByte(0x00000004), 0xDE, "First byte of word");
        runner.checkEqual<uint8_t>(mem.readByte(0x00000005), 0xBC, "Second byte of word");
        runner.checkEqual<uint8_t>(mem.readByte(0x00000006), 0x9A, "Third byte of word");
        runner.checkEqual<uint8_t>(mem.readByte(0x00000007), 0x78, "Fourth byte of word");
        
        // Check half-word access to word data
        runner.checkEqual<uint16_t>(mem.readHalf(0x00000004), 0xBCDE, "First half of word");
        runner.checkEqual<uint16_t>(mem.readHalf(0x00000006), 0x789A, "Second half of word");
        
        // Test writing/reading at the end of RAM
        mem.writeWord(0x001FFFFC, 0x12345678);
        runner.checkEqual<uint32_t>(mem.readWord(0x001FFFFC), 0x12345678, "Word at end of RAM");
    });
    
    // Test 3: Scratchpad RAM Access
    runner.addTest("Scratchpad RAM Access", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Write and read a byte
        mem.writeByte(0x1F800000, 0x12);
        runner.checkEqual<uint8_t>(mem.readByte(0x1F800000), 0x12, "Scratchpad byte write/read");
        
        // Write and read a half-word
        mem.writeHalf(0x1F800002, 0x3456);
        runner.checkEqual<uint16_t>(mem.readHalf(0x1F800002), 0x3456, "Scratchpad half-word write/read");
        
        // Write and read a word
        mem.writeWord(0x1F800004, 0x789ABCDE);
        runner.checkEqual<uint32_t>(mem.readWord(0x1F800004), 0x789ABCDE, "Scratchpad word write/read");
        
        // Test writing/reading at the end of scratchpad
        mem.writeWord(0x1F8003FC, 0x12345678);
        runner.checkEqual<uint32_t>(mem.readWord(0x1F8003FC), 0x12345678, "Word at end of scratchpad");
    });
    
    // Test 4: Memory-Mapped I/O
    runner.addTest("Memory-Mapped I/O", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Test I/O register callbacks
        uint32_t testIoReg = 0x55AAAA55;
        
        // Register a read handler for a test I/O port
        mem.registerIOReadHandler(0x1F801100, 4, [&testIoReg](uint32_t /*addr*/) -> uint32_t {
            return testIoReg;
        });
        
        // Register a write handler for the same port
        mem.registerIOWriteHandler(0x1F801100, 4, [&testIoReg](uint32_t /*addr*/, uint32_t value) {
            testIoReg = value;
        });
        
        // Test word access
        runner.checkEqual<uint32_t>(mem.readWord(0x1F801100), 0x55AAAA55, "I/O read word");
        
        mem.writeWord(0x1F801100, 0x12345678);
        runner.checkEqual<uint32_t>(testIoReg, 0x12345678, "I/O write word");
        runner.checkEqual<uint32_t>(mem.readWord(0x1F801100), 0x12345678, "I/O read after write");
        
        // Test byte access
        runner.checkEqual<uint8_t>(mem.readByte(0x1F801100), 0x78, "I/O read byte");
        
        mem.writeByte(0x1F801100, 0x99);
        runner.checkEqual<uint32_t>(testIoReg, 0x12345699, "I/O write byte");
        
        // Test half-word access
        runner.checkEqual<uint16_t>(mem.readHalf(0x1F801102), 0x1234, "I/O read half");
        
        mem.writeHalf(0x1F801102, 0xABCD);
        runner.checkEqual<uint32_t>(testIoReg, 0xABCD5699, "I/O write half");
    });
    
    // Test 5: Address Translation
    runner.addTest("Address Translation", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Write a value to KUSEG
        mem.writeWord(0x00100000, 0x11111111);
        
        // Read from KSEG0 (cached mirror)
        runner.checkEqual<uint32_t>(mem.readWord(0x80100000), 0x11111111, "KSEG0 mirroring");
        
        // Read from KSEG1 (uncached mirror)
        runner.checkEqual<uint32_t>(mem.readWord(0xA0100000), 0x11111111, "KSEG1 mirroring");
        
        // Write to KSEG0 and check if it's reflected in KUSEG
        mem.writeWord(0x80200000, 0x22222222);
        runner.checkEqual<uint32_t>(mem.readWord(0x00200000), 0x22222222, "KSEG0 to KUSEG mirroring");
        
        // Write to KSEG1 and check if it's reflected in KUSEG and KSEG0
        mem.writeWord(0xA0300000, 0x33333333);
        runner.checkEqual<uint32_t>(mem.readWord(0x00300000), 0x33333333, "KSEG1 to KUSEG mirroring");
        runner.checkEqual<uint32_t>(mem.readWord(0x80300000), 0x33333333, "KSEG1 to KSEG0 mirroring");
    });
    
    // Test 6: DMA Transfer
    runner.addTest("DMA Transfer", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Initialize source data
        for (int i = 0; i < 16; i++) {
            mem.writeWord(0x00000000 + i * 4, 0xAA000000 + i);
        }
        
        // Perform DMA transfer
        mem.dmaTransfer(0x00000000, 0x00100000, 64);
        
        // Verify transferred data
        for (int i = 0; i < 16; i++) {
            runner.checkEqual<uint32_t>(mem.readWord(0x00100000 + i * 4), 0xAA000000 + i, 
                                        "DMA transferred word " + std::to_string(i));
        }
    });
    
    // Test 7: Error Handling
    runner.addTest("Error Handling", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        
        // Test write to ROM (should be silently ignored)
        mem.writeWord(0x1FC00000, 0x12345678);
        runner.checkEqual<uint32_t>(mem.readWord(0x1FC00000), 0, "Write to ROM ignored");
        
        // Test unaligned access (should return 0xFFFFFFFF or 0xFFFF for unimplemented hardware)
        uint32_t unalignedWord = mem.readWord(0x00000001);  // Unaligned word read
        uint16_t unalignedHalf = mem.readHalf(0x00000001);  // Unaligned half-word read
        
        runner.check(unalignedWord == 0xFFFFFFFF, "Unaligned word read returns 0xFFFFFFFF");
        runner.check(unalignedHalf == 0xFFFF, "Unaligned half-word read returns 0xFFFF");
    });
    
    runner.runAll();
}

} // namespace PSXTest

int main() {
    std::cout << "Running PlayStation Memory Controller Tests\n";
    PSXTest::runMemoryControllerTests();
    return 0;
} 