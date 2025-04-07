#pragma once

#include <cstdint>
#include <functional>
#include <array>
#include <string>

namespace PSX {

// Class representing the PlayStation's CPU (MIPS architecture)
class CPU {
public:
    // Register indices for the 32 general purpose registers
    enum Register {
        R0, AT, V0, V1, A0, A1, A2, A3,
        T0, T1, T2, T3, T4, T5, T6, T7,
        S0, S1, S2, S3, S4, S5, S6, S7,
        T8, T9, K0, K1, GP, SP, FP, RA,
        REG_COUNT
    };
    
    // Names for the registers (for debugging)
    static const std::array<std::string, REG_COUNT> REG_NAMES;

    // Coprocessor 0 (System Control) registers
    enum CP0Register {
        CP0_INDEX = 0,     // Index into the TLB array
        CP0_RANDOM = 1,    // Randomly generated index into the TLB array
        CP0_ENTRYLO = 2,   // Low-order portion of the TLB entry
        CP0_CONTEXT = 3,   // Pointer to page table entry in memory
        CP0_BADVADDR = 8,  // Bad virtual address
        CP0_SR = 12,       // Status register
        CP0_CAUSE = 13,    // Cause of last exception
        CP0_EPC = 14,      // Exception program counter
        CP0_PRID = 15      // Processor revision identifier
    };

    // Exception types
    enum Exception {
        EXCEPTION_INTERRUPT = 0,
        EXCEPTION_TLB_MODIFIED = 1,
        EXCEPTION_TLB_LOAD = 2,
        EXCEPTION_TLB_STORE = 3,
        EXCEPTION_ADDRESS_ERROR_LOAD = 4,
        EXCEPTION_ADDRESS_ERROR_STORE = 5,
        EXCEPTION_BUS_ERROR_INSTRUCTION = 6,
        EXCEPTION_BUS_ERROR_DATA = 7,
        EXCEPTION_SYSCALL = 8,
        EXCEPTION_BREAKPOINT = 9,
        EXCEPTION_RESERVED_INSTRUCTION = 10,
        EXCEPTION_COPROCESSOR_UNUSABLE = 11,
        EXCEPTION_OVERFLOW = 12
    };

    // Status register bit masks
    static constexpr uint32_t SR_IEC = (1 << 0);     // Interrupt Enable (Current)
    static constexpr uint32_t SR_KUC = (1 << 1);     // Kernel/User mode (Current)
    static constexpr uint32_t SR_IEP = (1 << 2);     // Interrupt Enable (Previous)
    static constexpr uint32_t SR_KUP = (1 << 3);     // Kernel/User mode (Previous)
    static constexpr uint32_t SR_IEO = (1 << 4);     // Interrupt Enable (Old)
    static constexpr uint32_t SR_KUO = (1 << 5);     // Kernel/User mode (Old)
    static constexpr uint32_t SR_IM = (0xFF << 8);   // Interrupt Mask (8 bits)
    static constexpr uint32_t SR_ISC = (1 << 16);    // Isolate Cache
    static constexpr uint32_t SR_SWC = (1 << 17);    // Swap Caches
    static constexpr uint32_t SR_PZ = (1 << 18);     // Cache parity zero
    static constexpr uint32_t SR_CM = (1 << 19);     // Cache miss
    static constexpr uint32_t SR_PE = (1 << 20);     // Cache parity error
    static constexpr uint32_t SR_TS = (1 << 21);     // TLB shutdown
    static constexpr uint32_t SR_BEV = (1 << 22);    // Bootstrap exception vectors

    // Cause register bit masks
    static constexpr uint32_t CAUSE_EXCCODE = (0x1F << 2);  // Exception code
    static constexpr uint32_t CAUSE_IP = (0xFF << 8);       // Interrupt pending
    static constexpr uint32_t CAUSE_BD = (1 << 31);         // Branch delay slot

public:
    CPU();
    ~CPU();

    // Initialize/reset the CPU
    void reset();
    
    // Execute a single instruction
    void executeInstruction();
    
    // Execute a number of cycles
    void runCycles(uint32_t cycles);

    // Trigger an exception
    void triggerException(Exception exception, uint32_t pc);

    // Check for and handle pending interrupts
    void checkInterrupts();
    
    // Set an external interrupt
    void setInterrupt(int bit, bool asserted);

    // Memory access callbacks
    using MemoryReadCallback = std::function<uint32_t(uint32_t)>;
    using MemoryWriteCallback = std::function<void(uint32_t, uint32_t)>;
    
    // Set memory read/write callbacks
    void setMemoryCallbacks(MemoryReadCallback readCB, MemoryWriteCallback writeCB);

    // Memory access methods
    uint8_t readByte(uint32_t address);
    uint16_t readHalf(uint32_t address);
    uint32_t readWord(uint32_t address);
    void writeByte(uint32_t address, uint8_t value);
    void writeHalf(uint32_t address, uint16_t value);
    void writeWord(uint32_t address, uint32_t value);

    // Register access methods
    uint32_t getRegister(Register reg) const;
    void setRegister(Register reg, uint32_t value);
    
    uint32_t getCP0Register(CP0Register reg) const;
    void setCP0Register(CP0Register reg, uint32_t value);
    
    uint32_t getPC() const { return pc; }
    void setPC(uint32_t newPC) { pc = newPC; nextPc = newPC + 4; delaySlot = false; }
    
    uint32_t getHI() const { return hi; }
    void setHI(uint32_t value) { hi = value; }
    
    uint32_t getLO() const { return lo; }
    void setLO(uint32_t value) { lo = value; }
    
    // Debugging methods
    std::string disassembleInstruction(uint32_t instruction, uint32_t pc);
    void dumpState();

private:
    // Instruction execution helpers
    void decodeAndExecute(uint32_t instruction);
    
    // Instruction implementations
    void executeALU(uint32_t instruction);
    void executeJump(uint32_t instruction);
    void executeBranch(uint32_t instruction);
    void executeLoadStore(uint32_t instruction);
    void executeSpecial(uint32_t instruction);
    void executeRegImm(uint32_t instruction);
    void executeCoprocessor0(uint32_t instruction);
    void executeCoprocessor2(uint32_t instruction);

private:
    // CPU state
    std::array<uint32_t, REG_COUNT> gpr;  // General purpose registers
    uint32_t pc;                          // Program counter
    uint32_t nextPc;                      // Next instruction PC
    uint32_t hi;                          // High result register
    uint32_t lo;                          // Low result register
    std::array<uint32_t, 16> cp0;         // Coprocessor 0 registers
    bool delaySlot;                      // Whether we're in a branch delay slot
    
    // Memory interface
    MemoryReadCallback memoryRead;
    MemoryWriteCallback memoryWrite;
    
    // Statistics and runtime state
    uint64_t cycles;                      // Total executed cycles
    uint32_t currentInstruction;          // Currently executing instruction
    bool running;                         // CPU running state
};

} // namespace PSX