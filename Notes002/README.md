# PlayStation Coprocessor 0 (COP0) Implementation

The PlayStations's coprocessor 0 (COP0) is a crucial component responsible for system control, exception handling, and interrupt management. this document explains its functionality and provides practical examples of its usage. 

## Core Components

### 1. COP0 Registers

#### Status Register (Register 12)
```c++
// Key bits in Status Register
SR_IEC = (1 << 0)     // Interrupt Enable Current
SR_KUC = (1 << 1)     // Kernel/User Mode Current
SR_IEP = (1 << 2)     // Interrupt Enable Previous
SR_KUP = (1 << 3)     // Kernel/User Mode Previous
SR_IEO = (1 << 4)     // Interrupt Enable Old
SR_KUO = (1 << 5)     // Kernel/User Mode Old
SR_IM  = (0xFF << 8)  // Interrupt Mask
SR_BEV = (1 << 22)    // Bootstrap Exception Vectors
```

#### Cause Register (Register 13)
```c++
// Key fields in Cause Register
CAUSE_EXCCODE = (0x1F << 2) // Exception code
CAUSE_IP = (0xFF << 8)      // Interrupt pending
CAUSE_BD = (1 << 31)        // Branch delay slot
```

### 2. Exception Handling

The system supports various exception types:
```c++
EXCEPTION_INTERRUPT = 0
EXCEPTION_SYSCALL = 8
EXCEPTION_BREAKPOINT = 9
EXCEPTION_RESERVED_INSTRUCTION = 10
EXCEPTION_COPROCESSOR_UNUSABLE = 11
EXCEPTION_OVERFLOW = 12
```

### 3. Interrupt Controller

The PlayStation's interrupt controller is memory-mapped at the following addresses:
- I_STAT (0x1F801070) - Interrupt Status Register
- I_MASK (0x1F801074) - Interrupt Mask Register

Supported interrupt sources:
```c++
enum class InterruptType {
    VBLANK = 0,     // Vertical blank interrupt
    GPU = 1,        // GPU interrupt
    CDROM = 2,      // CD-ROM interrupt
    DMA = 3,        // DMA interrupt
    TIMER0 = 4,     // Timer 0 interrupt
    TIMER1 = 5,     // Timer 1 interrupt
    TIMER2 = 6,     // Timer 2 interrupt
    CONTROLLER = 7, // Controller interrupt
    SIO = 8,        // Serial I/O interrupt
    SPU = 9,        // Sound Processing Unit interrupt
};
```

## Real-World Examples

### 1. Handling Interrupts

When an interrupt occurs, the following sequence takes place:

```c++
// 1. Interrupt controller detects and signals interrupt
interruptController.trigger(InterruptType::VBLANK);

// 2. CPU checks for pending interrupts in each cycle
void CPU::checkInterrupts() {
    if ((cp0[CP0_SR] & SR_IEC) && 
        (cp0[CP0_SR] & cp0[CP0_CAUSE] & 0xFF00)) {
        triggerException(EXCEPTION_INTERRUPT, 0);
    }
}

// 3. Exception handler saves state and processes interrupt
void CPU::triggerException(Exception exception, uint32_t address) {
    // Save current PC in EPC
    cp0[CP0_EPC] = pc - (delaySlot ? 4 : 0);
    
    // Update status register (shift mode bits)
    uint32_t status = cp0[CP0_SR];
    uint32_t newStatus = (status & 0xFFFFFFE0) | 
                        ((status & 0x3) << 2) |    // Current → Previous
                        ((status >> 2) & 0x3) << 4; // Previous → Old
    cp0[CP0_SR] = newStatus;
    
    // Jump to exception vector
    pc = 0xBFC00180;
}
```

### 2. Returning from Exceptions (RFE)

The RFE instruction restores the processor state after handling an exception:

```c++
// RFE instruction implementation
void CPU::executeCoprocessor0(uint32_t instruction) {
    if ((instruction & 0x3F) == 0x10) {  // RFE
        uint32_t status = cp0[CP0_SR];
        // Restore mode bits:
        // Old → Previous, Previous → Current
        uint32_t newStatus = (status & 0xFFFFFFE0) |
                            ((status >> 4) & 0x3) << 2 |  // Old → Previous
                            ((status >> 2) & 0x3);        // Previous → Current
        cp0[CP0_SR] = newStatus;
    }
}
```

### 3. Interrupt Controller Integration

Example of how the interrupt controller integrates with the CPU:

```c++
// Set up interrupt handling
void setupInterrupts() {
    // 1. Enable interrupts in COP0
    cpu.setCP0Register(CPU::CP0_SR, CPU::SR_IEC | (1 << 8)); // IE and IM[0]
    
    // 2. Configure interrupt controller
    interruptController.setMask(1 << InterruptType::VBLANK);
    
    // 3. Set up callback
    interruptController.setInterruptCallback([&cpu](bool state) {
        cpu.setInterrupt(0, state);
    });
}
```

## Testing

The implementation includes comprehensive tests covering:
1. Exception handling
2. Interrupt processing
3. Register state management
4. Memory-mapped register access

See `cpu_core_tests.cpp` and `interrupt_controller_tests.cpp` for specific test cases.