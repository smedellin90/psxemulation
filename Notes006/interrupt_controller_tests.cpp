#include "interrupt_controller.h"
#include "cpu_core.h"
#include <iostream>
#include <map>
#include <vector>
#include <functional>

namespace PSXTest {

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

// Run the interrupt controller tests
void runInterruptControllerTests() {
    TestRunner runner;
    
    // Test 1: Basic Initialization
    runner.addTest("Interrupt Controller Initialization", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        
        // Connect the CPU to the interrupt controller
        ic.connectCPU(&cpu);
        
        runner.check(ic.getStatus() == 0, "Status should be initialized to 0");
        runner.check(ic.getMask() == 0, "Mask should be initialized to 0");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending initially");
        
        // Check CPU CAUSE register
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & PSX::CPU::CAUSE_IP) == 0, 
                    "CPU CAUSE register should have no interrupts set initially");
    });
    
    // Test 2: Triggering Interrupts
    runner.addTest("Triggering Interrupts", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        
        // Connect the CPU to the interrupt controller
        ic.connectCPU(&cpu);
        
        // Trigger VBLANK interrupt
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(ic.getStatus() == 0x1, "VBLANK interrupt bit should be set");
        runner.check(!ic.isInterruptPending(), "Interrupt shouldn't be pending when masked");
        
        // CPU interrupt line should not be set yet (masked)
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) == 0, 
                    "CPU CAUSE register should not have interrupt bit set when masked");
        
        // Set mask to enable VBLANK
        ic.setMask(0x1);
        runner.check(ic.isInterruptPending(), "Interrupt should be pending when enabled by mask");
        
        // CPU interrupt line should now be set
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) != 0, 
                    "CPU CAUSE register should have interrupt bit set when unmasked");
        
        // Trigger another interrupt (GPU)
        ic.trigger(PSX::InterruptController::InterruptType::GPU);
        runner.check(ic.getStatus() == 0x3, "Both VBLANK and GPU interrupt bits should be set");
        
        // Both CPU lines should be set
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & ((1 << 8) | (1 << 9))) == ((1 << 8) | (1 << 9)), 
                    "CPU CAUSE register should have both interrupt bits set");
        
        // Acknowledge VBLANK interrupt
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(ic.getStatus() == 0x2, "VBLANK interrupt bit should be cleared");
        
        // VBLANK line should be cleared, but GPU still set
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 8)) == 0, 
                    "CPU CAUSE register should have VBLANK bit cleared");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 9)) != 0, 
                    "CPU CAUSE register should still have GPU bit set");
        
        // Still pending because GPU is set and enabled
        runner.check(ic.isInterruptPending(), "GPU interrupt should still be pending");
        
        // Change mask to disable GPU
        ic.setMask(0x0);
        
        // Not pending anymore because all are masked
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending when all masked");
        
        // But CPU line for GPU still set (hardware behavior - lines reflect actual status)
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 9)) == 0, 
                    "CPU CAUSE register should have GPU bit cleared when masked");
    });
    
    // Test 3: Multiple Interrupts
    runner.addTest("Multiple Interrupts", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        
        // Connect the CPU to the interrupt controller
        ic.connectCPU(&cpu);
        
        // Set mask to enable various interrupts
        ic.setMask(0x1F); // Enable first 5 interrupts
        
        // Trigger several interrupts
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        ic.trigger(PSX::InterruptController::InterruptType::GPU);
        ic.trigger(PSX::InterruptController::InterruptType::TIMER0);
        
        // Check status and pending state
        runner.check(ic.getStatus() == 0x13, "Status should show all triggered interrupts");
        runner.check(ic.isInterruptPending(), "Interrupts should be pending");
        
        // Check CPU lines
        // VBLANK: Line 0 (bit 8), GPU: Line 1 (bit 9), TIMER0: Line 4 (bit 12)
        uint32_t expectedLines = (1 << 8) | (1 << 9) | (1 << 12);
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0xFF00) == expectedLines, 
                    "CPU CAUSE register should have correct interrupt bits set");
                    
        // Acknowledge all interrupts
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        ic.acknowledge(PSX::InterruptController::InterruptType::GPU);
        ic.acknowledge(PSX::InterruptController::InterruptType::TIMER0);
        
        // Check all cleared
        runner.check(ic.getStatus() == 0, "All interrupts should be cleared");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0xFF00) == 0, 
                    "CPU CAUSE register should have all interrupt bits cleared");
    });
    
    // Test 4: All Timers to Same CPU Line
    runner.addTest("Timer Interrupts", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        
        // Connect the CPU to the interrupt controller
        ic.connectCPU(&cpu);
        
        // Enable all timer interrupts
        ic.setMask(0x70); // Bits 4, 5, 6 for TIMER0, TIMER1, TIMER2
        
        // Trigger timer interrupts one by one
        ic.trigger(PSX::InterruptController::InterruptType::TIMER0);
        
        // Timer0 -> Line 4 (bit 12)
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 12)) != 0, 
                    "CPU CAUSE register should have TIMER line set for TIMER0");
                    
        ic.trigger(PSX::InterruptController::InterruptType::TIMER1);
        
        // Line still the same - all timers map to the same line
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 12)) != 0, 
                    "CPU CAUSE register should have TIMER line set for TIMER1");
                   
        // Acknowledge TIMER0
        ic.acknowledge(PSX::InterruptController::InterruptType::TIMER0);
        
        // Line still set because TIMER1 is still active
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 12)) != 0, 
                    "CPU CAUSE register should still have TIMER line set after TIMER0 acknowledgment");
                    
        // Acknowledge TIMER1
        ic.acknowledge(PSX::InterruptController::InterruptType::TIMER1);
        
        // Now the line should be clear
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & (1 << 12)) == 0, 
                    "CPU CAUSE register should have TIMER line clear after all timer acknowledgments");
    });
    
    // Test 5: Reset Function
    runner.addTest("Reset Function", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        
        // Connect the CPU to the interrupt controller
        ic.connectCPU(&cpu);
        
        // Setup some interrupts and mask
        ic.setMask(0x3);
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        ic.trigger(PSX::InterruptController::InterruptType::GPU);
        
        // Verify they're set
        runner.check(ic.getStatus() == 0x3, "Interrupts should be set before reset");
        runner.check(ic.getMask() == 0x3, "Mask should be set before reset");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & ((1 << 8) | (1 << 9))) != 0, 
                    "CPU CAUSE register should have interrupt bits set before reset");
        
        // Reset the controller
        ic.reset();
        
        // Verify all cleared
        runner.check(ic.getStatus() == 0, "Status should be 0 after reset");
        runner.check(ic.getMask() == 0, "Mask should be 0 after reset");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending after reset");
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0xFF00) == 0, 
                    "CPU CAUSE register should have no interrupts set after reset");
    });
    
    runner.runAll();
}

} // namespace PSXTest

int main() {
    std::cout << "Running PlayStation Interrupt Controller Tests\n";
    PSXTest::runInterruptControllerTests();
    return 0;
} 