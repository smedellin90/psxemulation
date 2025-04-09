#include "cpu_core.h"
#include "interrupt_controller.h"
#include "memory_controller.h"
#include <iostream>
#include <vector>
#include <functional>
#include <string>

// Test framework class
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
    
    // Fixed template - explicitly handle uint32_t
    bool checkEqual(uint32_t actual, uint32_t expected, const std::string& message) {
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

// Instruction encoding namespace
namespace Encode {
    namespace R_Type {
        uint32_t ADD(uint8_t rd, uint8_t rs, uint8_t rt) {
            return (rs << 21) | (rt << 16) | (rd << 11) | 0x20;
        }
        
        uint32_t SYSCALL() {
            return 0x0C;
        }
    }
    
    namespace I_Type {
        uint32_t ADDIU(uint8_t rt, uint8_t rs, uint16_t imm) {
            return (0x09 << 26) | (rs << 21) | (rt << 16) | imm;
        }
        
        uint32_t LW(uint8_t rt, uint8_t rs, uint16_t offset) {
            return (0x23 << 26) | (rs << 21) | (rt << 16) | offset;
        }
        
        uint32_t SW(uint8_t rt, uint8_t rs, uint16_t offset) {
            return (0x2B << 26) | (rs << 21) | (rt << 16) | offset;
        }
        
        uint32_t MTC0(uint8_t rt, uint8_t rd) {
            return (0x10 << 26) | (0x04 << 21) | (rt << 16) | (rd << 11);
        }
    }
    
    namespace J_Type {
        uint32_t J(uint32_t target) {
            return (0x02 << 26) | (target & 0x3FFFFFF);
        }
    }
}

// Test function for hardware-based interrupts
void runHardwareInterruptTests() {
    TestRunner runner;
    
    // Test 1: Hardware-based Interrupt Integration
    runner.addTest("Hardware-based Interrupt Integration", [&runner]() {
        // Setup CPU, Interrupt Controller, and Memory Controller
        PSX::CPU cpu;
        PSX::InterruptController ic;
        PSX::MemoryController mc(&ic);
        
        // Connect components
        ic.connectCPU(&cpu);
        cpu.setMemoryCallbacks(
            [&mc](uint32_t address) { return mc.readWord(address); },
            [&mc](uint32_t address, uint32_t value) { mc.writeWord(address, value); }
        );
        
        // Initially, no interrupts should be pending
        runner.check(ic.getStatus() == 0, "Interrupt status should start at 0");
        runner.check(ic.getMask() == 0, "Interrupt mask should start at 0");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending initially");
        
        // Check CPU CAUSE register has no interrupts set
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & PSX::CPU::CAUSE_IP) == 0, 
                    "CPU CAUSE register should have no interrupts initially");
        
        // Trigger VBLANK interrupt and enable it
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(ic.getStatus() == 0x1, "VBLANK interrupt status bit should be set");
        
        // Read status through memory-mapped register
        uint32_t status = mc.readWord(0x1F801070);
        runner.checkEqual(status, 0x1u, "Memory-mapped I_STAT register should return correct value");
        
        // Set interrupt mask through memory-mapped register
        mc.writeWord(0x1F801074, 0x1); // Enable VBLANK interrupts
        runner.checkEqual(ic.getMask(), 0x1u, "Memory-mapped I_MASK register should update correctly");
        
        // Now the interrupt should be pending and CPU line set
        runner.check(ic.isInterruptPending(), "Interrupt should be pending after enabling mask");
        uint32_t causeValue = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        uint32_t expectedBit = (1u << 8);
        runner.check((causeValue & expectedBit) != 0, 
                    "CPU CAUSE register should have VBLANK interrupt bit set");
        
        // Acknowledge interrupt through memory-mapped register
        mc.writeWord(0x1F801070, 0x1);
        runner.check(ic.getStatus() == 0, "VBLANK interrupt should be acknowledged");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending after acknowledgment");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1u << 8)) == 0, 
                    "CPU CAUSE register should have interrupt bit cleared");
    });
    
    // Test 2: Multiple Interrupts and CPU Line Mapping
    runner.addTest("Multiple Interrupts and CPU Line Mapping", [&runner]() {
        PSX::CPU cpu;
        PSX::InterruptController ic;
        ic.connectCPU(&cpu);
        
        // Enable several interrupts
        ic.setMask(0x1F); // Enable first 5 interrupts
        
        // Trigger several different interrupts
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);  // Maps to line 0 (bit 8)
        ic.trigger(PSX::InterruptController::InterruptType::GPU);     // Maps to line 1 (bit 9)
        ic.trigger(PSX::InterruptController::InterruptType::CDROM);   // Maps to line 2 (bit 10)
        
        // Check that all interrupts are pending
        runner.check(ic.isInterruptPending(), "Multiple interrupts should be pending");
        
        // Check that correct CPU lines are set in CAUSE register
        uint32_t cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1u << 8)) != 0, "CPU CAUSE register should have VBLANK line set");
        runner.check((cause & (1u << 9)) != 0, "CPU CAUSE register should have GPU line set");
        runner.check((cause & (1u << 10)) != 0, "CPU CAUSE register should have CDROM line set");
        
        // Acknowledge one interrupt at a time and check CPU lines
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1u << 8)) == 0, "VBLANK line should be cleared after acknowledgment");
        runner.check((cause & (1u << 9)) != 0, "GPU line should still be set");
        runner.check((cause & (1u << 10)) != 0, "CDROM line should still be set");
        
        ic.acknowledge(PSX::InterruptController::InterruptType::GPU);
        cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1u << 9)) == 0, "GPU line should be cleared after acknowledgment");
        runner.check((cause & (1u << 10)) != 0, "CDROM line should still be set");
        
        ic.acknowledge(PSX::InterruptController::InterruptType::CDROM);
        cause = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((cause & (1u << 10)) == 0, "CDROM line should be cleared after acknowledgment");
        
        // All lines should be clear now
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & PSX::CPU::CAUSE_IP) == 0, 
                    "All interrupt lines should be cleared");
    });
    
    // Test 3: Interrupt Masking at CPU Level
    runner.addTest("Interrupt Masking at CPU Level", [&runner]() {
        PSX::CPU cpu;
        PSX::InterruptController ic;
        ic.connectCPU(&cpu);
        
        // Enable interrupts in the interrupt controller
        ic.setMask(0x1); // Enable VBLANK
        
        // Disable interrupts in the CPU (SR.IE = 0)
        cpu.setCP0Register(PSX::CPU::CP0_SR, 0); // IE bit (bit 0) = 0
        
        // Trigger VBLANK
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        
        // The interrupt should be pending in the controller
        runner.check(ic.isInterruptPending(), "Interrupt should be pending in controller");
        
        // The CPU CAUSE register should show the interrupt
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1u << 8)) != 0, 
                    "CPU CAUSE register should show the interrupt");
        
        // But CPU should not take the interrupt because SR.IE = 0
        bool interruptTaken = false;
        // Call checkInterrupts manually and check if exception occurred
        uint32_t pcBefore = cpu.getPC();
        cpu.checkInterrupts();
        interruptTaken = cpu.getPC() != pcBefore;
        runner.check(!interruptTaken, "CPU should not take interrupt when IE = 0");
        
        // Enable interrupts in CPU (SR.IE = 1)
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1u << 8));
        
        // Now CPU should take the interrupt
        pcBefore = cpu.getPC();
        cpu.checkInterrupts();
        interruptTaken = cpu.getPC() != pcBefore;
        runner.check(interruptTaken, "CPU should take interrupt when IE = 1");
    });
    
    // Test 4: Program Execution with Interrupts - Simplified Version
    runner.addTest("Program Execution with Interrupts", [&runner]() {
        // Setup CPU, Interrupt Controller, and Memory Controller
        PSX::CPU cpu;
        PSX::InterruptController ic;
        PSX::MemoryController mc(&ic);
        
        // Connect components
        ic.connectCPU(&cpu);
        cpu.setMemoryCallbacks(
            [&mc](uint32_t address) { return mc.readWord(address); },
            [&mc](uint32_t address, uint32_t value) { mc.writeWord(address, value); }
        );
        
        // Define memory locations
        const uint32_t PROGRAM_START = 0x80000000;  // KSEG0 start (cacheable)
        
        // Simple counter program
        mc.writeWord(PROGRAM_START, Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::R0, 0));    // $2 = 0 (initialize counter)
        mc.writeWord(PROGRAM_START + 4, Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::V0, 1)); // $2++
        mc.writeWord(PROGRAM_START + 8, Encode::J_Type::J((PROGRAM_START + 4) >> 2));         // Loop
        
        // Setup CPU state
        cpu.reset();
        cpu.setPC(PROGRAM_START);
        
        // Run the first instruction (initialize counter)
        cpu.executeInstruction();
        
        // Verify counter initialized
        runner.checkEqual(cpu.getRegister(PSX::CPU::V0), 0u, "Counter should be initialized to 0");
        
        // Run a few iterations of the loop to increment counter
        for (int i = 0; i < 2; i++) {
            cpu.executeInstruction(); // ADDIU (increment)
            cpu.executeInstruction(); // Jump back
        }
        
        // Verify counter is incremented
        runner.checkEqual(cpu.getRegister(PSX::CPU::V0), 2u, "Counter should be incremented by loop");
        
        // Store CAUSE register value before triggering interrupt
        uint32_t causeBefore = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        
        // Enable interrupts and trigger one
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1u << 8)); // IE and IM[0]
        ic.setMask(0x1); // Enable VBLANK in controller
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        
        // Verify interrupt is pending in controller
        runner.check(ic.isInterruptPending(), "Interrupt should be pending in controller");
        
        // Verify CPU CAUSE register shows the interrupt
        uint32_t causeAfter = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((causeAfter & (1u << 8)) != 0, 
                    "CPU CAUSE register should show VBLANK interrupt");
        
        // Check if interrupt bits changed
        runner.check(causeBefore != causeAfter, 
                    "CAUSE register should have been updated when interrupt triggered");
                    
        // Manually call checkInterrupts to have CPU handle the exception
        uint32_t pcBeforeCheck = cpu.getPC();
        cpu.checkInterrupts();
        uint32_t pcAfterCheck = cpu.getPC();
        
        // The PC should change if exception was triggered
        runner.check(pcBeforeCheck != pcAfterCheck, 
                    "PC should change when interrupt is handled");
        
        // Acknowledge the interrupt
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        
        // Verify interrupt is acknowledged in controller
        runner.check(!ic.isInterruptPending(), "Interrupt should be cleared after acknowledgment");
        
        // Verify CPU CAUSE register shows cleared interrupt
        causeAfter = cpu.getCP0Register(PSX::CPU::CP0_CAUSE);
        runner.check((causeAfter & (1u << 8)) == 0, 
                    "CPU CAUSE register should show cleared VBLANK interrupt");
    });
    
    // Test 5: CPU Software Acknowledgment of Interrupts
    runner.addTest("CPU Software Acknowledgment of Interrupts", [&runner]() {
        // Setup CPU, Interrupt Controller, and Memory Controller
        PSX::CPU cpu;
        PSX::InterruptController ic;
        PSX::MemoryController mc(&ic);
        
        // Connect components
        ic.connectCPU(&cpu);

        // We'll manually intercept memory accesses to implement our own exception handler
        bool interruptHandled = false;
        uint32_t handlerCount = 0;

        // Create memory callback that will handle reading from exception vector
        cpu.setMemoryCallbacks(
            [&mc, &interruptHandled, &handlerCount](uint32_t address) {
                // When CPU tries to read from exception vector, return NOP instructions and acknowledge interrupt
                if (address >= 0xBFC00180 && address < 0xBFC00190 && !interruptHandled) {
                    if (address == 0xBFC00180) {
                        handlerCount++;
                        
                        // Acknowledge the interrupt
                        mc.writeWord(0x1F801070, 0x1);
                        
                        // Mark that we've handled it, so we don't handle it again
                        interruptHandled = true;
                        
                        // Return a NOP instruction (do nothing)
                        return 0u; // NOP instruction
                    }
                    
                    // For subsequent instructions in the exception handler, return a NOP
                    return 0u;
                }
                
                // Normal memory access
                return mc.readWord(address);
            },
            [&mc](uint32_t address, uint32_t value) { 
                mc.writeWord(address, value); 
            }
        );
        
        // Create a very simple program: 
        // - A counter initialization 
        // - A loop that increments the counter
        
        const uint32_t PROGRAM_START = 0x80000000;
        
        // Instruction: Set V0 = 0 (initialize counter)
        mc.writeWord(PROGRAM_START, Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::R0, 0)); 
        
        // Instruction: Set V0 = V0 + 1 (increment counter)
        mc.writeWord(PROGRAM_START + 4, Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::V0, 1));
        
        // Start CPU
        cpu.reset();
        cpu.setPC(PROGRAM_START);
        
        // Execute counter initialization
        cpu.executeInstruction();
        runner.checkEqual(cpu.getRegister(PSX::CPU::V0), 0u, "Counter should start at 0");
        runner.checkEqual(cpu.getPC(), PROGRAM_START + 4, "PC should point to increment instruction");
        
        // Enable interrupts
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1u << 8));  // IE bit and IM0 bit
        
        // Set up and trigger interrupt
        ic.setMask(0x1);  // Enable VBLANK
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        
        // Verify interrupt is pending
        runner.check(ic.isInterruptPending(), "VBLANK interrupt should be pending");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1u << 8)) != 0, 
                    "CPU CAUSE register should show interrupt");
        
        // Save current state
        uint32_t pcBeforeInterrupt = cpu.getPC();
        uint32_t counterBeforeInterrupt = cpu.getRegister(PSX::CPU::V0);
        
        // Trigger interrupt handling
        cpu.checkInterrupts();
        
        // Verify CPU jumped to exception vector
        runner.checkEqual(cpu.getPC(), 0xBFC00180u, "CPU should jump to exception vector");
        
        // Execute exception handler (4 NOPs)
        for (int i = 0; i < 4; i++) {
            cpu.executeInstruction();
        }
        
        // Verify interrupt was handled
        runner.checkEqual(handlerCount, 1u, "Interrupt handler should be executed");
        runner.check(!ic.isInterruptPending(), "Interrupt should be acknowledged");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1u << 8)) == 0, 
                    "CAUSE register should show interrupt cleared");
        
        // Return from exception to increment instruction
        cpu.setPC(pcBeforeInterrupt);
        
        // Verify we're back at the increment instruction
        runner.checkEqual(cpu.getPC(), PROGRAM_START + 4, "PC should return to increment instruction");
        
        // Execute the increment instruction
        cpu.executeInstruction();
        
        // Verify counter was incremented
        runner.checkEqual(cpu.getRegister(PSX::CPU::V0), counterBeforeInterrupt + 1, 
                         "Counter should be incremented after returning from exception");
    });
    
    runner.runAll();
}

int main() {
    std::cout << "Running PlayStation Hardware Interrupt Tests\n";
    runHardwareInterruptTests();
    return 0;
} 