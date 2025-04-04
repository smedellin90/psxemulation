#pragma once

#include <cstdint>
#include <functional>

namespace PSX {

// Interrupt Controller class for the PSX
class InterruptController {
public:
    // Interrupt types
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

    // Callback type for interrupt state changes
    using InterruptCallback = std::function<void(bool)>;

    // Constructor
    InterruptController();

    // Reset the interrupt controller
    void reset();

    // Check if an interrupt is pending
    bool isInterruptPending() const;

    // Get the current interrupt status
    uint32_t getStatus() const;

    // Get the current interrupt mask
    uint32_t getMask() const;

    // Set the interrupt mask
    void setMask(uint32_t mask);

    // Acknowledge an interrupt
    void acknowledge(InterruptType type);

    // Trigger an interrupt
    void trigger(InterruptType type);

    // Set callback for interrupt state changes
    void setInterruptCallback(InterruptCallback callback);

private:
    uint32_t status;  // Current interrupt status
    uint32_t mask;    // Interrupt mask
    uint32_t control; // Interrupt control register
    InterruptCallback onInterruptChange;  // Callback for interrupt state changes

    // Update interrupt state and call callback if needed
    void updateInterruptState();
};

} // namespace PSX 