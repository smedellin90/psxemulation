#include "interrupt_controller.h"
#include "cpu_core.h"
#include <iostream>
#include <functional>
#include <string>
#include <vector>
#include <map>

namespace PSXTest {

// Test framework class (simplified from cpu_core_tests.cpp)
class TestRunner {
private:
    struct TestCase {
        std::string name;
        std::function<void()> testFunc;
    };
    
    std::vector<TestCase> tests;
    int passed = 0;
    int failed = 0;
    bool currentTestPassed = true;
    std::string currentTestName;
    
public:
    void addTest(const std::string& name, std::function<void()> testFunc) {
        tests.push_back({name, testFunc});
    }
    
    void check(bool condition, const std::string& message) {
        if (!condition) {
            currentTestPassed = false;
            std::cout << "  FAIL: " << message << std::endl;
        }
    }
    
    void runAll() {
        for (const auto& test : tests) {
            currentTestName = test.name;
            currentTestPassed = true;
            
            std::cout << "Running test: " << test.name << std::endl;
            try {
                test.testFunc();
            } catch (const std::exception& e) {
                currentTestPassed = false;
                std::cout << "  EXCEPTION: " << e.what() << std::endl;
            } catch (...) {
                currentTestPassed = false;
                std::cout << "  UNKNOWN EXCEPTION" << std::endl;
            }
            
            if (currentTestPassed) {
                passed++;
                std::cout << "  PASSED" << std::endl;
            } else {
                failed++;
            }
        }
        
        std::cout << "\nTest Summary:" << std::endl;
        std::cout << std::dec << "  Total tests: " << tests.size() << std::endl;
        std::cout << "  Passed: " << passed << std::endl;
        std::cout << "  Failed: " << failed << std::endl;
    }
};

// Simple memory implementation for CPU tests
class TestMemory {
private:
    std::map<uint32_t, uint32_t> contents;
    PSX::InterruptController* interruptController;

public:
    TestMemory(PSX::InterruptController* ic) : interruptController(ic) {}
    
    uint32_t read(uint32_t address) {
        // Check for interrupt controller memory-mapped registers
        if (address == 0x1F801070) {
            return interruptController->getStatus();
        } else if (address == 0x1F801074) {
            return interruptController->getMask();
        }
        
        // Regular memory access
        address &= ~0x3; // Align to word boundary
        auto it = contents.find(address);
        if (it != contents.end()) {
            return it->second;
        }
        return 0; // Return 0 for uninitialized memory
    }

    void write(uint32_t address, uint32_t value) {
        // Check for interrupt controller memory-mapped registers
        if (address == 0x1F801070) {
            // Writing 1s to I_STAT acknowledges (clears) the corresponding interrupts
            // We need to determine which bits are being set to 1 and clear those interrupts
            uint32_t status = interruptController->getStatus();
            for (int i = 0; i < 10; i++) {
                if ((value >> i) & 1) {
                    interruptController->acknowledge(static_cast<PSX::InterruptController::InterruptType>(i));
                }
            }
            return;
        } else if (address == 0x1F801074) {
            interruptController->setMask(value);
            return;
        }
        
        // Regular memory access
        address &= ~0x3; // Align to word boundary
        contents[address] = value;
    }
};

// Run the interrupt controller tests
void runInterruptControllerTests() {
    TestRunner runner;
    
    // Test 1: Basic Initialization
    runner.addTest("Interrupt Controller Initialization", [&runner]() {
        PSX::InterruptController ic;
        
        runner.check(ic.getStatus() == 0, "Status should be initialized to 0");
        runner.check(ic.getMask() == 0, "Mask should be initialized to 0");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending initially");
    });
    
    // Test 2: Triggering Interrupts
    runner.addTest("Triggering Interrupts", [&runner]() {
        PSX::InterruptController ic;
        
        // Trigger VBLANK interrupt
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(ic.getStatus() == 0x1, "VBLANK interrupt bit should be set");
        runner.check(!ic.isInterruptPending(), "Interrupt shouldn't be pending when masked");
        
        // Set mask to enable VBLANK
        ic.setMask(0x1);
        runner.check(ic.isInterruptPending(), "Interrupt should be pending when enabled by mask");
        
        // Trigger another interrupt (GPU)
        ic.trigger(PSX::InterruptController::InterruptType::GPU);
        runner.check(ic.getStatus() == 0x3, "Both VBLANK and GPU interrupt bits should be set");
        
        // Acknowledge VBLANK interrupt
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(ic.getStatus() == 0x2, "VBLANK interrupt bit should be cleared");
        runner.check(!ic.isInterruptPending(), "No enabled interrupts should be pending");
        
        // Change mask to enable GPU
        ic.setMask(0x2);
        runner.check(ic.isInterruptPending(), "GPU interrupt should now be pending");
    });
    
    // Test 3: Multiple Interrupts
    runner.addTest("Multiple Interrupts", [&runner]() {
        PSX::InterruptController ic;
        
        // Trigger multiple interrupts
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        ic.trigger(PSX::InterruptController::InterruptType::TIMER0);
        ic.trigger(PSX::InterruptController::InterruptType::CDROM);
        
        // Expected status: bits 0, 2, and 4 set (VBLANK, CDROM, TIMER0)
        uint32_t expectedStatus = (1 << 0) | (1 << 2) | (1 << 4);
        runner.check(ic.getStatus() == expectedStatus, "Status should reflect all triggered interrupts");
        
        // No interrupts enabled yet
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending when all masked");
        
        // Enable TIMER0 only
        ic.setMask(1 << 4);
        runner.check(ic.isInterruptPending(), "TIMER0 interrupt should be pending");
        
        // Acknowledge TIMER0
        ic.acknowledge(PSX::InterruptController::InterruptType::TIMER0);
        runner.check(ic.getStatus() == ((1 << 0) | (1 << 2)), "TIMER0 bit should be cleared");
        runner.check(!ic.isInterruptPending(), "No interrupts should be pending after acknowledgment");
    });
    
    // Test 4: Interrupt Callback
    runner.addTest("Interrupt Callback", [&runner]() {
        PSX::InterruptController ic;
        bool callbackCalled = false;
        bool interruptState = false;
        
        // Set callback
        ic.setInterruptCallback([&callbackCalled, &interruptState](bool state) {
            callbackCalled = true;
            interruptState = state;
        });
        
        // Trigger an interrupt but keep it masked
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(callbackCalled, "Callback should be called on interrupt trigger");
        runner.check(!interruptState, "Interrupt state should be false when masked");
        
        // Reset for next test
        callbackCalled = false;
        
        // Enable the interrupt
        ic.setMask(0x1);
        runner.check(callbackCalled, "Callback should be called when mask changes");
        runner.check(interruptState, "Interrupt state should be true when unmasked");
        
        // Reset for next test
        callbackCalled = false;
        
        // Acknowledge the interrupt
        ic.acknowledge(PSX::InterruptController::InterruptType::VBLANK);
        runner.check(callbackCalled, "Callback should be called when interrupt acknowledged");
        runner.check(!interruptState, "Interrupt state should be false after acknowledgment");
    });
    
    // Test 5: Integration with CPU
    runner.addTest("Integration with CPU", [&runner]() {
        PSX::InterruptController ic;
        PSX::CPU cpu;
        TestMemory mem(&ic);
        
        // Setup CPU with memory
        cpu.setMemoryCallbacks(
            [&mem](uint32_t address) { return mem.read(address); },
            [&mem](uint32_t address, uint32_t value) { mem.write(address, value); }
        );
        cpu.reset();
        
        // Connect interrupt controller to CPU
        bool interruptTriggered = false;
        ic.setInterruptCallback([&cpu, &interruptTriggered](bool state) {
            cpu.setInterrupt(0, state); // Use bit 0 for simplicity
            interruptTriggered = state;
        });
        
        // Enable interrupts in CPU
        cpu.setCP0Register(PSX::CPU::CP0_SR, PSX::CPU::SR_IEC | (1 << 8)); // IE and IM[0]
        
        // Trigger an interrupt
        ic.setMask(0x1); // Enable VBLANK in interrupt controller
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        
        // Check if interrupt was triggered
        runner.check(interruptTriggered, "Interrupt should be triggered");
        
        // Check if CPU recognizes the interrupt
        cpu.checkInterrupts();
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & PSX::CPU::CAUSE_IP) != 0, 
                    "CPU CAUSE register should have IP bits set");
                    
        // Write to I_STAT to acknowledge interrupt
        mem.write(0x1F801070, 0x1); // Acknowledge VBLANK
        
        // Verify interrupt is cleared
        runner.check(!ic.isInterruptPending(), "Interrupt should be cleared after acknowledgment");
        runner.check(!interruptTriggered, "Interrupt callback should indicate no pending interrupt");
    });
    
    // Test 6: Memory-Mapped Registers
    runner.addTest("Memory-Mapped Registers", [&runner]() {
        PSX::InterruptController ic;
        TestMemory mem(&ic);
        
        // Set some interrupt state
        ic.trigger(PSX::InterruptController::InterruptType::VBLANK);
        ic.setMask(0x3); // Enable VBLANK and GPU
        
        // Read registers through memory interface
        uint32_t readStatus = mem.read(0x1F801070);
        uint32_t readMask = mem.read(0x1F801074);
        
        runner.check(readStatus == 0x1, "Reading I_STAT should return correct value");
        runner.check(readMask == 0x3, "Reading I_MASK should return correct value");
        
        // Write to registers through memory interface
        mem.write(0x1F801074, 0x5); // Enable VBLANK and TIMER0
        runner.check(ic.getMask() == 0x5, "Writing to I_MASK should update mask");
        
        // Acknowledge via memory write
        mem.write(0x1F801070, 0x1); // Acknowledge VBLANK
        runner.check(ic.getStatus() == 0x0, "Writing to I_STAT should acknowledge interrupts");
    });
    
    runner.runAll();
}

} // namespace PSXTest

int main() {
    std::cout << "Running PlayStation Interrupt Controller Tests\n";
    PSXTest::runInterruptControllerTests();
    return 0;
} 