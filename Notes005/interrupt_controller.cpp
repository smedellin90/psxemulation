#include "interrupt_controller.h"
#include "cpu_core.h"
#include <cassert>

namespace PSX {

InterruptController::InterruptController()
    : status(0)
    , mask(0)
    , control(0)
    , cpu(nullptr)
{
}

void InterruptController::connectCPU(CPU* cpuPtr) {
    cpu = cpuPtr;
}

void InterruptController::reset() {
    status = 0;
    mask = 0;
    control = 0;
    
    // Reset all interrupt lines on the CPU
    if (cpu) {
        for (int i = 0; i < 8; i++) {
            cpu->setInterrupt(i, false);
        }
    }
}

bool InterruptController::isInterruptPending() const {
    return (status & mask) != 0;
}

uint32_t InterruptController::getStatus() const {
    return status;
}

uint32_t InterruptController::getMask() const {
    return mask;
}

void InterruptController::setMask(uint32_t newMask) {
    mask = newMask;
    updateCPUInterrupts();
}

void InterruptController::acknowledge(InterruptType type) {
    status &= ~(1 << static_cast<uint32_t>(type));
    updateCPUInterrupts();
}

void InterruptController::trigger(InterruptType type) {
    status |= (1 << static_cast<uint32_t>(type));
    updateCPUInterrupts();
}

InterruptController::CPUInterruptLine InterruptController::mapInterruptToCPULine(InterruptType type) {
    // Map each PlayStation interrupt type to the appropriate CPU interrupt line
    // This mapping is based on the PlayStation hardware architecture
    switch (type) {
        case InterruptType::VBLANK:
            return CPUInterruptLine::VBLANK_LINE;
        case InterruptType::GPU:
            return CPUInterruptLine::GPU_LINE;
        case InterruptType::CDROM:
            return CPUInterruptLine::CDROM_LINE;
        case InterruptType::DMA:
            return CPUInterruptLine::DMA_LINE;
        case InterruptType::TIMER0:
        case InterruptType::TIMER1:
        case InterruptType::TIMER2:
            return CPUInterruptLine::TIMER_LINE;
        case InterruptType::CONTROLLER:
            return CPUInterruptLine::CONTROLLER_LINE;
        case InterruptType::SIO:
            return CPUInterruptLine::SIO_LINE;
        case InterruptType::SPU:
            return CPUInterruptLine::SPU_LINE;
        default:
            // If unknown, default to VBLANK line (should never happen)
            return CPUInterruptLine::VBLANK_LINE;
    }
}

void InterruptController::updateCPUInterrupts() {
    if (!cpu) return;
    
    // Clear all interrupt lines first
    for (int i = 0; i < 8; i++) {
        cpu->setInterrupt(i, false);
    }
    
    // For each active interrupt, set the appropriate CPU interrupt line
    for (uint32_t i = 0; i < 10; i++) {
        if ((status & mask & (1 << i)) != 0) {
            InterruptType type = static_cast<InterruptType>(i);
            CPUInterruptLine line = mapInterruptToCPULine(type);
            cpu->setInterrupt(static_cast<int>(line), true);
        }
    }
    
    // The CPU will check its CAUSE register against its Status register mask
    // during instruction execution and trigger an exception if needed
}

} // namespace PSX 