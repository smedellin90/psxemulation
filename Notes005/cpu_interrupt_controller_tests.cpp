#include "cpu_core.h"
#include "interrupt_controller.h"
#include "memory_controller.h"
#include <cstdint>
#include <iostream>
#include <functional>
#include <map>
#include <vector>
#include <string>

namespace PSXTest {

// Simple test runner class
class TestRunner {
private:
    struct Test {
        std::string name;
        std::function<void()> testFunc;
    };
    
    std::vector<Test> tests;
    int passCount = 0;
    int failCount = 0;
    std::string currentTest;

public:
    void addTest(const std::string& name, std::function<void()> testFunc) {
        tests.push_back({name, testFunc});
    }
    
    bool check(bool condition, const std::string& message) {
        if (condition) {
            std::cout << "  ✓ " << message << std::endl;
            passCount++;
            return true;
        } else {
            std::cout << "  ✗ " << message << std::endl;
            failCount++;
            return false;
        }
    }
    
    template<typename T>
    bool checkEqual(const T& actual, const T& expected, const std::string& message) {
        if (actual == expected) {
            std::cout << "  ✓ " << message << " (Got: 0x" << std::hex << actual << std::dec << ")" << std::endl;
            passCount++;
            return true;
        } else {
            std::cout << "  ✗ " << message << " (Expected: 0x" << std::hex << expected 
                      << ", Got: 0x" << actual << std::dec << ")" << std::endl;
            failCount++;
            return false;
        }
    }
    
    void runAll() {
        for (const auto& test : tests) {
            std::cout << "Running test: " << test.name << std::endl;
            currentTest = test.name;
            test.testFunc();
            std::cout << std::endl;
        }
        
        std::cout << "=============================================" << std::endl;
        std::cout << "Test Results: " << passCount << " passed, " << failCount << " failed, " 
                  << (passCount + failCount) << " total" << std::endl;
        std::cout << "=============================================" << std::endl;
    }
};

// Helper function to setup interrupt controller mappings in memory controller
void setupInterruptControllerMapping(PSX::MemoryController& memoryController, PSX::InterruptController& interruptController) {
    // Map I_STAT (Interrupt Status) to 0x1F801070
    memoryController.registerIOReadHandler(0x1F801070, 4, [&interruptController](uint32_t /*address*/) {
        return interruptController.getStatus();
    });
    
    memoryController.registerIOWriteHandler(0x1F801070, 4, [&interruptController](uint32_t /*address*/, uint32_t value) {
        // Writing 1s to I_STAT acknowledges (clears) the corresponding interrupts
        for (int i = 0; i < 10; i++) {
            if ((value >> i) & 1) {
                interruptController.acknowledge(static_cast<PSX::InterruptController::InterruptType>(i));
            }
        }
    });
    
    // Map I_MASK (Interrupt Mask) to 0x1F801074
    memoryController.registerIOReadHandler(0x1F801074, 4, [&interruptController](uint32_t /*address*/) {
        return interruptController.getMask();
    });
    
    memoryController.registerIOWriteHandler(0x1F801074, 4, [&interruptController](uint32_t /*address*/, uint32_t value) {
        interruptController.setMask(value);
    });
    
    // Also map the remaining interrupt registers to prevent "Unhandled I/O read" warnings
    // These are not used in our tests but might be accessed by the memory controller
    memoryController.registerIOReadHandler(0x1F801078, 4, [](uint32_t /*address*/) {
        return 0; // Default value for unused registers
    });
    
    memoryController.registerIOReadHandler(0x1F80107C, 4, [](uint32_t /*address*/) {
        return 0; // Default value for unused registers
    });
}

// Simple namespace for MIPS instruction encoding
namespace Encode {
    namespace R_Type {
        uint32_t ADDU(int rd, int rs, int rt) {
            return (0 << 26) | (rs << 21) | (rt << 16) | (rd << 11) | (0 << 6) | 0x21;
        }
    }
    
    namespace I_Type {
        uint32_t ADDIU(int rt, int rs, uint16_t imm) {
            return (9 << 26) | (rs << 21) | (rt << 16) | imm;
        }
        
        uint32_t LW(int rt, int base, int16_t offset) {
            return (0x23 << 26) | (base << 21) | (rt << 16) | (offset & 0xFFFF);
        }
        
        uint32_t SW(int rt, int base, int16_t offset) {
            return (0x2B << 26) | (base << 21) | (rt << 16) | (offset & 0xFFFF);
        }
        
        uint32_t BEQ(int rs, int rt, int16_t offset) {
            return (0x4 << 26) | (rs << 21) | (rt << 16) | (offset & 0xFFFF);
        }
        
        uint32_t BNE(int rs, int rt, int16_t offset) {
            return (0x5 << 26) | (rs << 21) | (rt << 16) | (offset & 0xFFFF);
        }
        
        uint32_t LUI(int rt, uint16_t imm) {
            return (0xF << 26) | (0 << 21) | (rt << 16) | imm;
        }
        
        uint32_t ORI(int rt, int rs, uint16_t imm) {
            return (0xD << 26) | (rs << 21) | (rt << 16) | imm;
        }
    }
    
    namespace J_Type {
        uint32_t J(uint32_t target) {
            return (0x2 << 26) | ((target >> 2) & 0x3FFFFFF);
        }
        
        uint32_t JAL(uint32_t target) {
            return (0x3 << 26) | ((target >> 2) & 0x3FFFFFF);
        }
    }
}

// Run the CPU and Interrupt Controller integration tests
void runCPUInterruptControllerTests() {
    TestRunner runner;
    
    // Test 1: Basic Interrupt Integration
    runner.addTest("Basic Interrupt Integration", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        PSX::CPU cpu;
        
        // Setup interrupt controller mappings in memory controller
        setupInterruptControllerMapping(mem, ic);
        
        // Setup CPU with memory
        cpu.setMemoryCallbacks(
            [&mem](uint32_t address) { return mem.readWord(address); },
            [&mem](uint32_t address, uint32_t value) { mem.writeWord(address, value); }
        );
        cpu.reset();
        
        // Connect interrupt controller to CPU (direct hardware connection)
        ic.connectCPU(&cpu);
        
        // Enable interrupts in CPU
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1 << 8)); // IE and IM[0]
        
        // Trigger an interrupt
        ic.setMask(0x1); // Enable VBLANK in interrupt controller
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        
        // Check if interrupt is visible in CAUSE register
        uint32_t cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1 << 8)) != 0, "VBLANK interrupt should be set in CAUSE register");
        
        // Check if CPU recognizes the interrupt
        cpu.checkInterrupts();
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & PSX::CPU::CAUSE_IP) != 0, 
                    "CPU CAUSE register should have IP bits set");
                    
        // Acknowledge interrupt via memory-mapped register
        mem.writeWord(0x1F801070, 0x1); // Acknowledge VBLANK
        
        // Verify interrupt is cleared
        runner.check(!ic.isInterruptPending(), "Interrupt should be cleared after acknowledgment");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) == 0, 
                    "CAUSE register should show interrupt cleared");
    });
    
    // Test 2: Multiple Interrupts
    runner.addTest("Multiple Interrupts", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        PSX::CPU cpu;
        
        // Setup interrupt controller mappings in memory controller
        setupInterruptControllerMapping(mem, ic);
        
        // Setup CPU with memory and connect interrupt controller
        cpu.setMemoryCallbacks(
            [&mem](uint32_t address) { return mem.readWord(address); },
            [&mem](uint32_t address, uint32_t value) { mem.writeWord(address, value); }
        );
        cpu.reset();
        
        // Connect interrupt controller to CPU
        ic.connectCPU(&cpu);
        
        // Enable interrupts in CPU for specific sources
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (3 << 8)); // IE and IM[0-1]
        
        // Set mask in interrupt controller for VBLANK and GPU
        ic.setMask(0x3);
        
        // Trigger multiple interrupts
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        ic.trigger(PSX::InterruptController::InterruptType::GPU);
        
        // Check if interrupts are pending
        runner.check(ic.isInterruptPending(), "Multiple interrupts should be pending");
        
        // Check CPU interrupt state
        cpu.checkInterrupts();
        uint32_t cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1 << 8)) != 0, "VBLANK interrupt should be set in CAUSE register");
        runner.check((cause & (1 << 9)) != 0, "GPU interrupt should be set in CAUSE register");
        
        // Acknowledge only one interrupt
        mem.writeWord(0x1F801070, 0x1); // Acknowledge only VBLANK
        
        // Check that interrupt is still pending (due to GPU)
        runner.check(ic.isInterruptPending(), "Interrupt should still be pending after partial acknowledgment");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) == 0, "VBLANK interrupt should be cleared in CAUSE");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 9)) != 0, "GPU interrupt should still be set in CAUSE");
        
        // Acknowledge the other interrupt
        mem.writeWord(0x1F801070, 0x2); // Acknowledge GPU
        
        // Verify all interrupts are cleared
        runner.check(!ic.isInterruptPending(), "All interrupts should be cleared after full acknowledgment");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & ((1 << 8) | (1 << 9))) == 0, 
                     "All interrupt bits should be cleared in CAUSE register");
    });
    
    // Test 3: Interrupt Handling with Instruction Execution
    runner.addTest("Interrupt Handling with Instruction Execution", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        PSX::CPU cpu;
        
        // Setup interrupt controller mappings in memory controller
        setupInterruptControllerMapping(mem, ic);
        
        // Setup CPU with memory
        cpu.setMemoryCallbacks(
            [&mem](uint32_t address) { return mem.readWord(address); },
            [&mem](uint32_t address, uint32_t value) { mem.writeWord(address, value); }
        );
        cpu.reset();
        
        // Connect interrupt controller to CPU
        ic.connectCPU(&cpu);
        
        // Verify initial register state
        runner.checkEqual(cpu.getRegister(PSX::CPU::T0), 0u, "Register T0 should be initially zero");
        
        // Setup a simple program
        uint32_t programStart = 0x80001000;
        uint32_t physAddr = 0x00001000; // Corresponding physical address
        
        // Main program: Add 10 to register $t0
        uint32_t instr = Encode::I_Type::ADDIU(PSX::CPU::T0, PSX::CPU::R0, 10);
        mem.writeWord(physAddr, instr);
        
        // Verify instruction was written correctly
        runner.checkEqual(mem.readWord(physAddr), instr, "Instruction should be stored correctly in memory");
        
        // Set PC to start of program
        cpu.setPC(programStart);
        
        // Verify CPU can read the instruction
        uint32_t fetched = cpu.fetchInstruction(programStart);
        runner.checkEqual(fetched, instr, "CPU should fetch correct instruction from memory");
        
        // Now execute the instruction
        cpu.executeInstruction();
        
        // Check that T0 has the expected value after executing the instruction
        runner.checkEqual(cpu.getRegister(PSX::CPU::T0), 10u, "Register T0 should be set to 10");
        
        // Enable interrupts in CPU
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1 << 8)); // IE and IM[0]
        
        // Trigger an interrupt
        ic.setMask(0x1);
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        
        // Check interrupt pending state
        runner.check(ic.isInterruptPending(), "Interrupt should be pending");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) != 0, 
                     "VBLANK interrupt should be set in CAUSE register");
        
        // Manually check for interrupts
        cpu.checkInterrupts();
        
        // Check if the CPU recognized the interrupt and triggered an exception
        // We expect the exception to cause a jump to the exception vector address
        runner.checkEqual(cpu.getPC(), 0xBFC00180u, "PC should be at the exception vector address");
        
        // The exception code should be set for an interrupt
        uint32_t cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        uint32_t exceptionType = (cause & PSX::CPU::CAUSE_EXCCODE) >> 2;
        runner.checkEqual(exceptionType, (uint32_t)PSX::CPU::EXCEPTION_INTERRUPT, 
                        "Exception type should be INTERRUPT");
    });
    
    // Test 4: Reading and Writing Interrupt Registers
    runner.addTest("Reading and Writing Interrupt Registers", [&runner]() {
        PSX::InterruptController ic;
        PSX::MemoryController mem(&ic);
        PSX::CPU cpu;
        
        // Setup interrupt controller mappings in memory controller
        setupInterruptControllerMapping(mem, ic);
        
        // Setup CPU with memory
        cpu.setMemoryCallbacks(
            [&mem](uint32_t address) { return mem.readWord(address); },
            [&mem](uint32_t address, uint32_t value) { mem.writeWord(address, value); }
        );
        
        // Connect CPU to interrupt controller
        ic.connectCPU(&cpu);
        
        // Trigger some interrupts
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        ic.trigger(PSX::InterruptController::InterruptType::TIMER0);
        
        // Set a mask
        ic.setMask(0x11); // VBLANK and TIMER0
        
        // Enable all interrupts in CPU
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | 0xFF00); // IE and all IMs
        
        // Read interrupt status register through CPU memory interface
        uint32_t status = cpu.readWord(0x1F801070);
        runner.checkEqual(status, 0x11u, "CPU should read correct I_STAT value");
        
        // Read interrupt mask register through CPU memory interface
        uint32_t mask = cpu.readWord(0x1F801074);
        runner.checkEqual(mask, 0x11u, "CPU should read correct I_MASK value");
        
        // Verify interrupts are pending and visible in CAUSE register
        runner.check(ic.isInterruptPending(), "Interrupts should be pending");
        uint32_t cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1 << 8)) != 0, "VBLANK interrupt should be set in CAUSE register");
        runner.check((cause & (1 << 12)) != 0, "TIMER0 interrupt should be set in CAUSE register");
        
        // Acknowledge VBLANK through CPU memory interface
        cpu.writeWord(0x1F801070, 0x1);
        
        // Verify VBLANK was acknowledged but TIMER0 is still pending
        runner.check(ic.isInterruptPending(), "TIMER0 interrupt should still be pending");
        cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1 << 8)) == 0, "VBLANK interrupt should be cleared in CAUSE register");
        runner.check((cause & (1 << 12)) != 0, "TIMER0 interrupt should still be set in CAUSE register");
        
        // Acknowledge TIMER0 through CPU memory interface
        cpu.writeWord(0x1F801070, 0x10);
        
        // Verify all interrupts are cleared
        runner.check(!ic.isInterruptPending(), "All interrupts should be cleared");
        cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & ((1 << 8) | (1 << 12))) == 0, "All interrupt bits should be cleared in CAUSE register");
    });
    
    // Test 5: Interrupt Priority and Masking
    runner.addTest("Interrupt Priority and Masking", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        
        // Connect interrupt controller to CPU
        ic.connectCPU(&cpu);
        
        // Enable interrupts in CPU
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1 << 8)); // IE and IM[0]
        
        // Initially no interrupts pending
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending initially");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) == 0, 
                     "No interrupts should be set in CAUSE register initially");
        
        // Trigger an interrupt but keep it masked (mask is 0 by default)
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(!ic.isInterruptPending(), "Masked interrupt should not be pending");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) == 0, 
                     "Masked interrupt should not set bit in CAUSE register");
        
        // Enable the interrupt
        ic.setMask(0x1);
        runner.check(ic.isInterruptPending(), "Enabled interrupt should be pending");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) != 0, 
                     "Enabled interrupt should set bit in CAUSE register");
        
        // Trigger a higher priority interrupt
        ic.trigger(PSX::InterruptController::InterruptType::CDROM);
        
        // Acknowledge VBLANK
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        
        // CDROM is still pending but masked
        runner.check(!ic.isInterruptPending(), "Higher priority interrupt should not be pending if masked");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 10)) == 0, 
                     "Masked CDROM interrupt should not set bit in CAUSE register");
        
        // Enable CDROM interrupt
        ic.setMask(0x4);
        runner.check(ic.isInterruptPending(), "Newly enabled interrupt should be pending");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 10)) != 0, 
                     "Enabled CDROM interrupt should set bit in CAUSE register");
        
        // Acknowledge all interrupts
        ic.acknowledge(PSX::InterruptController::InterruptType::CDROM);
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending after acknowledgment");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & ((1 << 8) | (1 << 10))) == 0, 
                     "All interrupt bits should be cleared in CAUSE register");
    });
    
    runner.runAll();
}

} // namespace PSXTest

int main() {
    std::cout << "Running PlayStation CPU/Interrupt Controller Integration Tests\n";
    PSXTest::runCPUInterruptControllerTests();
    return 0;
} 