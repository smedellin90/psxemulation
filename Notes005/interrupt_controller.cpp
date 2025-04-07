#include "interrupt_controller.h"
#include <cassert>

namespace PSX {

InterruptController::InterruptController()
    : status(0)
    , mask(0)
    , control(0)
    , onInterruptChange(nullptr)
{
}

void InterruptController::reset() {
    status = 0;
    mask = 0;
    control = 0;
    if (onInterruptChange) {
        onInterruptChange(false);
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
    updateInterruptState();
}

void InterruptController::acknowledge(InterruptType type) {
    status &= ~(1 << static_cast<uint32_t>(type));
    updateInterruptState();
}

void InterruptController::trigger(InterruptType type) {
    status |= (1 << static_cast<uint32_t>(type));
    updateInterruptState();
}

void InterruptController::setInterruptCallback(InterruptCallback callback) {
    onInterruptChange = callback;
    updateInterruptState();
}

void InterruptController::updateInterruptState() {
    if (onInterruptChange) {
        onInterruptChange(isInterruptPending());
    }
}

} // namespace PSX 