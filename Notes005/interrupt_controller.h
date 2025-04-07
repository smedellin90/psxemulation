#pragma once

#include <cstdint>
#include <functional>

namespace PSX {

// Forward declaration of CPU class
class CPU;

// Interrupt Controller class for the PSX
class InterruptController {
public:
    // Interrupt types and their corresponding bit positions
    enum class InterruptType : uint32_t {
        VBLANK = 0,      // Vertical blank interrupt
        GPU = 1,         // GPU interrupt
        CDROM = 2,       // CD-ROM interrupt
        DMA = 3,         // DMA interrupt
        TIMER0 = 4,      // Timer 0 interrupt
        TIMER1 = 5,      // Timer 1 interrupt
        TIMER2 = 6,      // Timer 2 interrupt
        CONTROLLER = 7,  // Controller interrupt
        SIO = 8,         // Serial I/O interrupt
        SPU = 9,         // Sound Processing Unit interrupt
    };

    // CPU interrupt line mapping
    // Each PlayStation interrupt type is mapped to a specific CPU interrupt line
    // Based on PlayStation hardware architecture
    enum class CPUInterruptLine : uint32_t {
        VBLANK_LINE = 0,       // Line 0 for VBLANK
        GPU_LINE = 1,          // Line 1 for GPU
        CDROM_LINE = 2,        // Line 2 for CDROM
        DMA_LINE = 3,          // Line 3 for DMA
        TIMER_LINE = 4,        // Line 4 for all Timer interrupts (0,1,2)
        CONTROLLER_LINE = 5,   // Line 5 for Controller
        SIO_LINE = 6,          // Line 6 for SIO
        SPU_LINE = 7           // Line 7 for SPU
    };

    // Constructor
    InterruptController();

    // Connect CPU to the interrupt controller
    void connectCPU(CPU* cpu);

    // Reset the interrupt controller
    void reset();

    // Check if an interrupt is pending
    bool isInterruptPending() const;

    // Get the current interrupt status
    uint32_t getStatus() const;

    // Get the current interrupt mask
    uint32_t getMask() const;

    // Set the interrupt mask and update CPU interrupts
    void setMask(uint32_t mask);

    // Acknowledge an interrupt and update CPU
    void acknowledge(InterruptType type);

    // Trigger an interrupt and update CPU
    void trigger(InterruptType type);

private:
    // Memory-mapped registers
    uint32_t status;   // I_STAT - Current interrupt status
    uint32_t mask;     // I_MASK - Interrupt mask
    uint32_t control;  // I_CTRL - Interrupt control register

    // Direct reference to the CPU (no callbacks needed anymore)
    CPU* cpu;
    
    // Maps PlayStation interrupt types to CPU interrupt lines
    static CPUInterruptLine mapInterruptToCPULine(InterruptType type);
    
    // Update the CPU's interrupt lines based on current status and mask
    void updateCPUInterrupts();
};

} // namespace PSX 