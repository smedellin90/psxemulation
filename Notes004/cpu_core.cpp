#include "cpu_core.h"
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <array>
#include <string>

namespace PSX {

// Register names for debugging
const std::array<std::string, CPU::REG_COUNT> CPU::REG_NAMES = {
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra"
};

CPU::CPU()
    : pc(0), nextPc(0), hi(0), lo(0), delaySlot(false),
      memoryRead(nullptr), memoryWrite(nullptr),
      iCacheEnabled(true), dCacheEnabled(true),
      cycles(0), currentInstruction(0), running(false) {
    // Initialize cache structures
    iCache.resize(ICACHE_LINES);
    dCache.resize(DCACHE_LINES);
    
    reset();
}

CPU::~CPU() {
    // Cleanup if needed
}

void CPU::reset() {
    // Clear registers
    for (auto& reg : gpr) {
        reg = 0;
    }
    
    // R0 is hardwired to 0
    gpr[R0] = 0;
    
    // Clear CP0 registers
    for (auto& reg : cp0) {
        reg = 0;
    }
    
    // Set up initial state
    pc = 0xBFC00000;  // BIOS entry point
    nextPc = pc + 4;
    hi = 0;
    lo = 0;
    delaySlot = false;
    cycles = 0;
    currentInstruction = 0;
    running = true;
    
    // Reset caches
    invalidateICache();
    invalidateDCache();
    resetCacheStats();
    
    // Enable caches by default
    iCacheEnabled = true;
    dCacheEnabled = true;
    
    // Set processor ID register
    cp0[CP0_PRID] = 0x00000002;  // CPU processor
    
    // Set status register initial value
    cp0[CP0_SR] = SR_BEV;  // Boot exception vectors in BIOS
}

void CPU::setMemoryCallbacks(MemoryReadCallback readCB, MemoryWriteCallback writeCB) {
    memoryRead = readCB;
    memoryWrite = writeCB;
}

uint32_t CPU::getRegister(Register reg) const {
    return gpr[reg];
}

void CPU::setRegister(Register reg, uint32_t value) {
    if (reg != R0) {  // R0 is always 0
        gpr[reg] = value;
    }
}

uint32_t CPU::getCP0Register(CP0Register reg) const {
    return cp0[reg];
}

void CPU::setCP0Register(CP0Register reg, uint32_t value) {
    // Special handling for Status Register
    if (reg == CP0_SR) {
        // Extract cache control bits
        bool isolate = (value & SR_ISC) != 0;
        bool swap = (value & SR_SWC) != 0;
        isolateCache(isolate);
        swapCaches(swap);
    }
    
    cp0[reg] = value;
}

uint32_t CPU::readMemoryDirect(uint32_t address) {
    if (!memoryRead) {
        std::cerr << "Memory read callback not set" << std::endl;
        return 0;
    }
    return memoryRead(address);
}

void CPU::writeMemoryDirect(uint32_t address, uint32_t value) {
    if (!memoryWrite) {
        std::cerr << "Memory write callback not set" << std::endl;
        return;
    }
    memoryWrite(address, value);
}

uint8_t CPU::readByte(uint32_t address) {
    uint32_t alignedAddr = address & ~0x3;
    uint32_t wordData;
    
    // Check if data is in cache
    if (dCacheEnabled && !isDCacheEnabled()) {
        // If data cache is isolated, only access cache, not memory
        if (!checkDCache(alignedAddr, wordData)) {
            return 0xFF; // Cache miss in isolated mode returns garbage
        }
    } else if (dCacheEnabled && checkDCache(alignedAddr, wordData)) {
        // Cache hit
        cacheStats.dCacheHits++;
    } else {
        // Cache miss or cache disabled
        if (dCacheEnabled) {
            cacheStats.dCacheMisses++;
        }
        wordData = readMemoryDirect(alignedAddr);
        if (dCacheEnabled) {
            updateDCache(alignedAddr, wordData);
        }
    }
    
    // Extract appropriate byte
    int byteOffset = address & 0x3;
    return (wordData >> (byteOffset * 8)) & 0xFF;
}

uint16_t CPU::readHalf(uint32_t address) {
    if (address & 0x1) {
        // Handle address error exception
        triggerException(EXCEPTION_ADDRESS_ERROR_LOAD, address);
        return 0;
    }
    
    uint32_t alignedAddr = address & ~0x3;
    uint32_t wordData;
    
    // Check if data is in cache
    if (dCacheEnabled && !isDCacheEnabled()) {
        // If data cache is isolated, only access cache, not memory
        if (!checkDCache(alignedAddr, wordData)) {
            return 0xFFFF; // Cache miss in isolated mode returns garbage
        }
    } else if (dCacheEnabled && checkDCache(alignedAddr, wordData)) {
        // Cache hit
        cacheStats.dCacheHits++;
    } else {
        // Cache miss or cache disabled
        if (dCacheEnabled) {
            cacheStats.dCacheMisses++;
        }
        wordData = readMemoryDirect(alignedAddr);
        if (dCacheEnabled) {
            updateDCache(alignedAddr, wordData);
        }
    }
    
    // Extract appropriate half-word
    int halfOffset = (address & 0x2) >> 1;
    return (wordData >> (halfOffset * 16)) & 0xFFFF;
}

uint32_t CPU::readWord(uint32_t address) {
    if (address & 0x3) {
        // Handle address error exception
        triggerException(EXCEPTION_ADDRESS_ERROR_LOAD, address);
        return 0;
    }
    
    uint32_t wordData;
    
    // Check if data is in cache
    if (dCacheEnabled && !isDCacheEnabled()) {
        // If data cache is isolated, only access cache, not memory
        if (!checkDCache(address, wordData)) {
            return 0xFFFFFFFF; // Cache miss in isolated mode returns garbage
        }
    } else if (dCacheEnabled && checkDCache(address, wordData)) {
        // Cache hit
        cacheStats.dCacheHits++;
    } else {
        // Cache miss or cache disabled
        if (dCacheEnabled) {
            cacheStats.dCacheMisses++;
        }
        wordData = readMemoryDirect(address);
        if (dCacheEnabled) {
            updateDCache(address, wordData);
        }
    }
    
    return wordData;
}

void CPU::writeByte(uint32_t address, uint8_t value) {
    uint32_t alignedAddr = address & ~0x3;
    uint32_t wordData;
    
    if (dCacheEnabled && checkDCache(alignedAddr, wordData)) {
        // If in cache, update it
        int byteOffset = address & 0x3;
        uint32_t mask = ~(0xFF << (byteOffset * 8));
        wordData = (wordData & mask) | (static_cast<uint32_t>(value) << (byteOffset * 8));
        updateDCache(alignedAddr, wordData);
    }
    
    // Always write to memory (write-through policy)
    wordData = readMemoryDirect(alignedAddr);
    int byteOffset = address & 0x3;
    uint32_t mask = ~(0xFF << (byteOffset * 8));
    wordData = (wordData & mask) | (static_cast<uint32_t>(value) << (byteOffset * 8));
    writeMemoryDirect(alignedAddr, wordData);
}

void CPU::writeHalf(uint32_t address, uint16_t value) {
    if (address % 2 != 0) {
        // Handle address error exception
        triggerException(EXCEPTION_ADDRESS_ERROR_STORE, address);
        return;
    }
    
    uint32_t alignedAddr = address & ~0x3;
    uint32_t wordData;
    
    if (dCacheEnabled && checkDCache(alignedAddr, wordData)) {
        // If in cache, update it
        int halfOffset = (address & 0x2) >> 1;
        uint32_t mask = ~(0xFFFF << (halfOffset * 16));
        wordData = (wordData & mask) | (static_cast<uint32_t>(value) << (halfOffset * 16));
        updateDCache(alignedAddr, wordData);
    }
    
    // Always write to memory (write-through policy)
    wordData = readMemoryDirect(alignedAddr);
    int halfOffset = (address & 0x2) >> 1;
    uint32_t mask = ~(0xFFFF << (halfOffset * 16));
    wordData = (wordData & mask) | (static_cast<uint32_t>(value) << (halfOffset * 16));
    writeMemoryDirect(alignedAddr, wordData);
}

void CPU::writeWord(uint32_t address, uint32_t value) {
    if (address % 4 != 0) {
        // Handle address error exception
        triggerException(EXCEPTION_ADDRESS_ERROR_STORE, address);
        return;
    }
    
    if (dCacheEnabled) {
        // Update cache if enabled (write-through)
        updateDCache(address, value);
    }
    
    // Always write to memory
    writeMemoryDirect(address, value);
}

uint32_t CPU::fetchInstruction(uint32_t address) {
    if (address & 0x3) {
        // Handle address error exception
        triggerException(EXCEPTION_ADDRESS_ERROR_LOAD, address);
        return 0;
    }
    
    uint32_t instructionData;
    
    // Check if instruction is in cache
    if (iCacheEnabled && !isICacheEnabled()) {
        // If instruction cache is isolated, only access cache, not memory
        if (!checkICache(address, instructionData)) {
            return 0; // Cache miss in isolated mode returns garbage
        }
    } else if (iCacheEnabled && checkICache(address, instructionData)) {
        // Cache hit
        cacheStats.iCacheHits++;
    } else {
        // Cache miss or cache disabled
        if (iCacheEnabled) {
            cacheStats.iCacheMisses++;
        }
        instructionData = readMemoryDirect(address);
        if (iCacheEnabled) {
            updateICache(address, instructionData);
        }
    }
    
    return instructionData;
}

void CPU::invalidateICache() {
    for (auto& line : iCache) {
        line.valid = false;
    }
}

void CPU::invalidateDCache() {
    for (auto& line : dCache) {
        line.valid = false;
    }
}

void CPU::isolateCache(bool isolate) {
    if (isolate) {
        cp0[CP0_SR] |= SR_ISC;
    } else {
        cp0[CP0_SR] &= ~SR_ISC;
    }
}

void CPU::swapCaches(bool swap) {
    if (swap) {
        cp0[CP0_SR] |= SR_SWC;
        // When swapped, ICache acts as DCache and vice versa
        // This affects the isCacheEnabled() checks
    } else {
        cp0[CP0_SR] &= ~SR_SWC;
    }
}

bool CPU::isCacheEnabled() const {
    return iCacheEnabled || dCacheEnabled;
}

bool CPU::isICacheEnabled() const {
    bool swapped = (cp0[CP0_SR] & SR_SWC) != 0;
    bool isolated = (cp0[CP0_SR] & SR_ISC) != 0;
    
    // When swapped, check opposite cache's enabled status
    return swapped ? dCacheEnabled : (iCacheEnabled && !isolated);
}

bool CPU::isDCacheEnabled() const {
    bool swapped = (cp0[CP0_SR] & SR_SWC) != 0;
    bool isolated = (cp0[CP0_SR] & SR_ISC) != 0;
    
    // When swapped, check opposite cache's enabled status
    return swapped ? iCacheEnabled : (dCacheEnabled && !isolated);
}

void CPU::resetCacheStats() {
    cacheStats.reset();
}

CPU::CacheStats CPU::getCacheStats() const {
    return cacheStats;
}

bool CPU::checkICache(uint32_t address, uint32_t& data) {
    // Only check cached memory regions (KSEG0 and KUSEG)
    if ((address & 0xE0000000) == 0xA0000000) {
        return false; // KSEG1 is uncached
    }
    
    uint32_t tag = address >> 4;
    uint32_t index = (address >> 4) & (ICACHE_LINES - 1);
    
    // Check if cache line is valid and tag matches
    if (iCache[index].valid && iCache[index].tag == tag) {
        // Assemble the word from the cache line
        uint32_t offset = address & 0xF;
        data = 0;
        for (int i = 0; i < 4; i++) {
            data |= static_cast<uint32_t>(iCache[index].data[offset + i]) << (i * 8);
        }
        return true;
    }
    
    return false;
}

bool CPU::checkDCache(uint32_t address, uint32_t& data) {
    // Only check cached memory regions (KSEG0 and KUSEG)
    if ((address & 0xE0000000) == 0xA0000000) {
        return false; // KSEG1 is uncached
    }
    
    uint32_t tag = address >> 4;
    uint32_t index = (address >> 4) & (DCACHE_LINES - 1);
    
    // Check if cache line is valid and tag matches
    if (dCache[index].valid && dCache[index].tag == tag) {
        // Assemble the word from the cache line
        uint32_t offset = address & 0xF;
        data = 0;
        for (int i = 0; i < 4; i++) {
            data |= static_cast<uint32_t>(dCache[index].data[offset + i]) << (i * 8);
        }
        return true;
    }
    
    return false;
}

void CPU::updateICache(uint32_t address, uint32_t data) {
    // Only cache specific memory regions (KSEG0 and KUSEG)
    if ((address & 0xE0000000) == 0xA0000000) {
        return; // KSEG1 is uncached
    }
    
    uint32_t tag = address >> 4;
    uint32_t index = (address >> 4) & (ICACHE_LINES - 1);
    uint32_t offset = address & 0xF;
    
    // If we're updating a partial cache line or it's invalid, load the entire line from memory
    if (!iCache[index].valid || iCache[index].tag != tag) {
        // Load the entire cache line from memory
        uint32_t lineStartAddr = address & ~0xF;
        for (int i = 0; i < CACHE_LINE_SIZE; i += 4) {
            uint32_t wordData = readMemoryDirect(lineStartAddr + i);
            for (int j = 0; j < 4; j++) {
                iCache[index].data[i + j] = (wordData >> (j * 8)) & 0xFF;
            }
        }
        
        iCache[index].tag = tag;
        iCache[index].valid = true;
    } else {
        // Just update the specific word
        for (int i = 0; i < 4; i++) {
            iCache[index].data[offset + i] = (data >> (i * 8)) & 0xFF;
        }
    }
}

void CPU::updateDCache(uint32_t address, uint32_t data) {
    // Only cache specific memory regions (KSEG0 and KUSEG)
    if ((address & 0xE0000000) == 0xA0000000) {
        return; // KSEG1 is uncached
    }
    
    uint32_t tag = address >> 4;
    uint32_t index = (address >> 4) & (DCACHE_LINES - 1);
    uint32_t offset = address & 0xF;
    
    // If we're updating a partial cache line or it's invalid, load the entire line from memory
    if (!dCache[index].valid || dCache[index].tag != tag) {
        // Load the entire cache line from memory
        uint32_t lineStartAddr = address & ~0xF;
        for (int i = 0; i < CACHE_LINE_SIZE; i += 4) {
            uint32_t wordData = readMemoryDirect(lineStartAddr + i);
            for (int j = 0; j < 4; j++) {
                dCache[index].data[i + j] = (wordData >> (j * 8)) & 0xFF;
            }
        }
        
        dCache[index].tag = tag;
        dCache[index].valid = true;
    }
    
    // Update the specific word
    for (int i = 0; i < 4; i++) {
        dCache[index].data[offset + i] = (data >> (i * 8)) & 0xFF;
    }
}

void CPU::executeInstruction() {
    if (!running) {
        return;
    }

    // Fetch instruction from memory using instruction cache
    currentInstruction = fetchInstruction(pc);

    // Remember if we're in a delay slot for this instruction
    bool wasInDelaySlot = delaySlot;

    // Save branch target if we're in delay slot
    uint32_t branchTarget = nextPc;
    
    // Update PC values before executing
    pc = nextPc; // Advance PC to next instruction. If a branch/jump modifies nextPc below,
                 // the current instruction (delay slot) will still execute before transferring control. 
    nextPc = nextPc + 4;

    // Reset delay slot flag - instructions will set it again if needed.
    delaySlot = false;
    
    // Execute the instruction
    decodeAndExecute(currentInstruction);

    // if we were in a delay slot, we need to jump to the branch target now
    if (wasInDelaySlot) {
        pc = branchTarget;
        nextPc = branchTarget + 4;
    }
    
    // Increment cycle count
    cycles++;

    // Check for interrupts
    checkInterrupts();
}

void CPU::runCycles(uint32_t count) {
    for (uint32_t i = 0; i < count && running; i++) {
        executeInstruction();
    }
}

void CPU::triggerException(Exception exception, uint32_t address) {
    // Save current status register
    uint32_t status = cp0[CP0_SR];
    
    // Shift mode bits:
    // Current → Previous
    // Previous → Old
    uint32_t newStatus = status & 0xFFFFFFE0;  // Clear mode bits
    newStatus |= ((status & 0x3) << 2);        // Current → Previous
    newStatus |= ((status >> 2) & 0x3) << 4;   // Previous → Old
    newStatus |= 0x0;                          // Set current mode to kernel (00)
    
    // Store updated status register
    cp0[CP0_SR] = newStatus;
    
    // For SYSCALL and BREAKPOINT, we need to use the PC value before it was incremented
    // since the PC has already been advanced to the next instruction during executeInstruction
    if (exception == EXCEPTION_SYSCALL || exception == EXCEPTION_BREAKPOINT) {
        cp0[CP0_EPC] = pc - 4;  // PC was already advanced, so we need to save the current instruction address
    } else {
        // For other exceptions, we use the current PC and adjust if in a delay slot
        cp0[CP0_EPC] = pc - (delaySlot ? 4 : 0);
    }
    
    // Set cause register
    cp0[CP0_CAUSE] &= ~CAUSE_EXCCODE;
    cp0[CP0_CAUSE] |= (static_cast<uint32_t>(exception) << 2);
    
    // Set BD bit if in delay slot
    if (delaySlot) {
        cp0[CP0_CAUSE] |= CAUSE_BD;
    } else {
        cp0[CP0_CAUSE] &= ~CAUSE_BD;
    }
    
    // Set BadVAddr if applicable
    if (exception == EXCEPTION_ADDRESS_ERROR_LOAD || 
        exception == EXCEPTION_ADDRESS_ERROR_STORE) {
        cp0[CP0_BADVADDR] = address;
    }
    
    // Jump to exception vector at 0xBFC00180
    pc = 0xBFC00180;
    nextPc = pc + 4;
    delaySlot = false;
}

void CPU::checkInterrupts() {
    // Check if interrupts are enabled
    if ((cp0[CP0_SR] & SR_IEC) && 
        (cp0[CP0_SR] & cp0[CP0_CAUSE] & 0xFF00)) {
        triggerException(EXCEPTION_INTERRUPT, 0);
    }
}

void CPU::setInterrupt(int bit, bool asserted) {
    if (bit >= 0 && bit <= 7) {
        if (asserted) {
            cp0[CP0_CAUSE] |= (1 << (bit + 8));
        } else {
            cp0[CP0_CAUSE] &= ~(1 << (bit + 8));
        }
    }
}

void CPU::decodeAndExecute(uint32_t instruction) {
    uint32_t opcode = instruction >> 26;
    
    switch (opcode) {
        case 0: executeSpecial(instruction); break; // SPECIAL
        case 1: executeRegImm(instruction); break;  // REGIMM
        case 2:  // J
        case 3:  // JAL
            executeJump(instruction);
            break;
        case 4:  // BEQ
        case 5:  // BNE
        case 6:  // BLEZ
        case 7:  // BGTZ
            executeBranch(instruction);
            break;
        case 8:  // ADDI
        case 9:  // ADDIU
        case 10: // SLTI
        case 11: // SLTIU
        case 12: // ANDI
        case 13: // ORI
        case 14: // XORI
        case 15: // LUI
            executeALU(instruction);
            break;
        case 16: // COP0
            executeCoprocessor0(instruction);
            break;
        case 18: // COP2
            executeCoprocessor2(instruction);
            break;
        case 32: // LB
        case 33: // LH
        case 34: // LWL
        case 35: // LW
        case 36: // LBU
        case 37: // LHU
        case 38: // LWR
        case 40: // SB
        case 41: // SH
        case 42: // SWL
        case 43: // SW
        case 46: // SWR
            executeLoadStore(instruction);
            break;
        default:
            // Unimplemented or invalid instruction
            std::cerr << "Unimplemented opcode: " << std::hex << opcode << std::endl;
            triggerException(EXCEPTION_RESERVED_INSTRUCTION, 0);
            break;
    }
}

void CPU::dumpState() {
    std::cout << "CPU State:" << std::endl;
    std::cout << "PC = 0x" << std::hex << std::setw(8) << std::setfill('0') << pc << std::endl;

    for (int i = 0; i < REG_COUNT; i += 4) {
        for (int j = 0; j < 4 && (i + j) < REG_COUNT; j++) {
            std::cout << std::setw(4) << std::left << REG_NAMES[i + j] << ": 0x" 
                      << std::hex << std::setw(8) << std::setfill('0') << std::right 
                      << gpr[i + j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "HI: 0x" << std::hex << std::setw(8) << std::setfill('0') << hi
              << " LO: 0x" << std::hex << std::setw(8) << std::setfill('0') << lo << std::endl;
              
    std::cout << "CP0 Status: 0x" << std::hex << std::setw(8) << std::setfill('0') << cp0[CP0_SR]
              << " Cause: 0x" << std::hex << std::setw(8) << std::setfill('0') << cp0[CP0_CAUSE] << std::endl;
              
    std::cout << "Current instruction: 0x" << std::hex << std::setw(8) << std::setfill('0') 
              << currentInstruction << std::endl;
}

// Placeholder for instruction implementations
// These would be implemented with the actual instruction logic
void CPU::executeALU(uint32_t instruction) {
    // Extract fields from instruction
    uint32_t opcode = (instruction >> 26) & 0x3F;    // Upper 6 bits
    uint32_t rs = (instruction >> 21) & 0x1F;        // Source register (5 bits)
    uint32_t rt = (instruction >> 16) & 0x1F;        // Target register (5 bits)
    uint16_t imm = instruction & 0xFFFF;             // Immediate value (16 bits)
    int16_t simm = static_cast<int16_t>(imm);        // Signed immediate

    // Get source register value
    uint32_t rsVal = gpr[rs];

    // Result to be written to rt
    uint32_t result = 0;

    // Process based on opcode
    switch (opcode) {
        case 8:  // ADDI - Add Immediate (with overflow check)
            {
                int32_t rs_signed = static_cast<int32_t>(rsVal);
                int32_t sum = rs_signed + simm;
                
                // Check for overflow
                bool overflow = ((rs_signed >= 0 && simm >= 0 && sum < 0) || 
                                (rs_signed < 0 && simm < 0 && sum >= 0));
                
                if (overflow) {
                    triggerException(EXCEPTION_OVERFLOW, 0);
                    return;
                }
                
                result = static_cast<uint32_t>(sum);
            }
            break;
            
        case 9:  // ADDIU - Add Immediate Unsigned (no overflow check)
            // Note: "unsigned" is a misnomer - the operation is still signed
            // but does not trigger overflow exceptions
            result = rsVal + simm;
            break;
            
        case 10: // SLTI - Set Less Than Immediate
            {
                int32_t rs_signed = static_cast<int32_t>(rsVal);
                result = (rs_signed < simm) ? 1 : 0;
            }
            break;
            
        case 11: // SLTIU - Set Less Than Immediate Unsigned
            // Note: Sign-extend immediate value first, then compare as unsigned
            {
                int32_t extended = simm; // Sign extension happens automatically
                result = (rsVal < static_cast<uint32_t>(extended)) ? 1 : 0;
            }
            break;
            
        case 12: // ANDI - AND Immediate
            // Note: No sign extension for logical operations
            result = rsVal & imm;
            break;
            
        case 13: // ORI - OR Immediate
            // Note: No sign extension for logical operations
            result = rsVal | imm;
            break;
            
        case 14: // XORI - XOR Immediate
            // Note: No sign extension for logical operations
            result = rsVal ^ imm;
            break;
            
        case 15: // LUI - Load Upper Immediate
            // Loads immediate value into upper 16 bits of register
            result = imm << 16;
            break;
    }

    // Write result to destination register
    setRegister(static_cast<Register>(rt), result);
}

void CPU::executeJump(uint32_t instruction) {
    uint32_t opcode = (instruction >> 26) & 0x3F;
    uint32_t target = instruction & 0x03FFFFFF;  // 26-bit jump target address
    
    // Calculate the jump target address
    // Jump address = top 4 bits of current PC + target address shifted left 2 bits
    uint32_t jumpAddress = (pc & 0xF0000000) | (target << 2);
    
    if (opcode == 3) {  // JAL - Jump and Link
        // Save return address in register 31 (RA)
        setRegister(RA, pc + 4);  // Fixed: Return address should be pc + 4 (which is original_pc + 8)
    }
    
    // Set up delay slot handling
    delaySlot = true;
    nextPc = jumpAddress;
}

void CPU::executeBranch(uint32_t instruction) {
    uint32_t opcode = (instruction >> 26) & 0x3F;
    uint32_t rs = (instruction >> 21) & 0x1F;
    uint32_t rt = (instruction >> 16) & 0x1F;
    int16_t offset = static_cast<int16_t>(instruction & 0xFFFF);
    
    // Calculate potential branch target address (delay slot address + offset << 2)
    uint32_t targetAddress = pc + (offset << 2);
    bool takeBranch = false;
    
    // Evaluate branch condition based on opcode
    switch (opcode) {
        case 4:  // BEQ - Branch if Equal
            takeBranch = (gpr[rs] == gpr[rt]);
            break;
            
        case 5:  // BNE - Branch if Not Equal
            takeBranch = (gpr[rs] != gpr[rt]);
            break;
            
        case 6:  // BLEZ - Branch if Less Than or Equal to Zero
            takeBranch = (static_cast<int32_t>(gpr[rs]) <= 0);
            break;
            
        case 7:  // BGTZ - Branch if Greater Than Zero
            takeBranch = (static_cast<int32_t>(gpr[rs]) > 0);
            break;
    }
    
    // If branch condition is met, set nextPc to branch target
    if (takeBranch) {
        delaySlot = true;
        nextPc = targetAddress;
    }
}

void CPU::executeLoadStore(uint32_t instruction) {
    uint32_t opcode = (instruction >> 26) & 0x3F;
    uint32_t rs = (instruction >> 21) & 0x1F;  // Base register
    uint32_t rt = (instruction >> 16) & 0x1F;  // Target register
    int16_t offset = static_cast<int16_t>(instruction & 0xFFFF);
    
    // Calculate effective address
    uint32_t address = gpr[rs] + offset;
    
    // Handle load instructions
    switch (opcode) {
        // Load instructions
        case 32:  // LB - Load Byte
            {
                int8_t value = static_cast<int8_t>(readByte(address));
                setRegister(static_cast<Register>(rt), static_cast<uint32_t>(value)); // Sign extend
            }
            break;
            
        case 33:  // LH - Load Halfword
            {
                int16_t value = static_cast<int16_t>(readHalf(address));
                setRegister(static_cast<Register>(rt), static_cast<uint32_t>(value)); // Sign extend
            }
            break;
            
        case 35:  // LW - Load Word
            setRegister(static_cast<Register>(rt), readWord(address));
            break;
            
        case 36:  // LBU - Load Byte Unsigned
            setRegister(static_cast<Register>(rt), readByte(address));
            break;
            
        case 37:  // LHU - Load Halfword Unsigned
            setRegister(static_cast<Register>(rt), readHalf(address));
            break;
            
        // Store instructions
        case 40:  // SB - Store Byte
            writeByte(address, gpr[rt] & 0xFF);
            break;
            
        case 41:  // SH - Store Halfword
            writeHalf(address, gpr[rt] & 0xFFFF);
            break;
            
        case 43:  // SW - Store Word
            writeWord(address, gpr[rt]);
            break;
            
        case 34:  // LWL - Load Word Left
            {
                // Get the aligned word containing the first byte
                uint32_t alignedAddress = address & ~0x3;
                uint32_t wordValue = memoryRead(alignedAddress);
                uint32_t origRt = gpr[rt];
                
                // Calculate offset (0-3)
                uint8_t offset = address & 0x3;
                
                // Handle each offset explicitly to ensure correct behavior
                uint32_t result;
                switch (offset) {
                    case 0:  // Aligned - replace entire register
                        result = wordValue;
                        break;
                    case 1:  // Replace upper 3 bytes, keep lowest byte
                        result = ((wordValue & 0x00FFFFFF) << 8) | (origRt & 0xFF);
                        break;
                    case 2:  // Replace upper 2 bytes, keep lowest 2 bytes
                        result = ((wordValue & 0x0000FFFF) << 16) | (origRt & 0xFFFF);
                        break;
                    case 3:  // Replace uppermost byte, keep lowest 3 bytes
                        result = ((wordValue & 0x000000FF) << 24) | (origRt & 0xFFFFFF);
                        break;
                }
                
                // Debug output removed for cleaner test output
                
                setRegister(static_cast<Register>(rt), result);
            }
            break;
            
        case 38:  // LWR - Load Word Right
            {
                // Get the aligned word containing the last byte
                uint32_t alignedAddress = address & ~0x3;
                uint32_t wordValue = memoryRead(alignedAddress);
                uint32_t origRt = gpr[rt];
                
                // Calculate offset (0-3)
                uint8_t offset = address & 0x3;
                
                // Byte-by-byte extraction from memory (0x44332211)
                uint8_t byte0 = wordValue & 0xFF;         // 0x11
                uint8_t byte1 = (wordValue >> 8) & 0xFF;  // 0x22
                uint8_t byte2 = (wordValue >> 16) & 0xFF; // 0x33
                uint8_t byte3 = (wordValue >> 24) & 0xFF; // 0x44
                
                uint32_t result;
                switch (offset) {
                    case 0:  // Replace lowest byte, keep upper 3 bytes
                        // origRt = 0xAABBCCDD, result = 0xAABBCC11
                        result = (origRt & 0xFFFFFF00) | byte0;
                        break;
                    case 1:  // Replace lowest 2 bytes, keep upper 2 bytes
                        // origRt = 0xAABBCCDD, result = 0xAABB2211
                        result = (origRt & 0xFFFF0000) | (byte1 << 8) | byte0;
                        break;
                    case 2:  // Replace lowest 3 bytes, keep uppermost byte
                        // origRt = 0xAABBCCDD, result = 0xAA332211
                        result = (origRt & 0xFF000000) | (byte2 << 16) | (byte1 << 8) | byte0;
                        break;
                    case 3:  // Aligned - replace entire register
                        // origRt = 0xAABBCCDD, result = 0x44332211
                        result = wordValue;
                        break;
                }
                
                // Debug output removed for cleaner test output
                
                setRegister(static_cast<Register>(rt), result);
            }
            break;
            
        case 42:  // SWL - Store Word Left
            {
                // Get the aligned address
                uint32_t alignedAddress = address & ~0x3;
                
                // Read the current memory value
                uint32_t oldValue = memoryRead(alignedAddress);
                uint32_t newValue;
                
                // Determine which bytes to store based on the address
                switch (address & 0x3) {
                    case 0:  // Aligned - store the entire word
                        newValue = gpr[rt];
                        break;
                    case 1:  // Store bytes 1-3 of register to bytes 0-2 of memory
                        newValue = ((gpr[rt] & 0xFFFFFF00) >> 8) | // Reg bytes 1-3 to mem bytes 0-2
                                  (oldValue & 0xFF000000);        // Keep byte 3 from memory
                        break;
                    case 2:  // Store bytes 2-3 of register to bytes 0-1 of memory
                        newValue = ((gpr[rt] & 0xFFFF0000) >> 16) | // Reg bytes 2-3 to mem bytes 0-1
                                  (oldValue & 0xFFFF0000);         // Keep bytes 2-3 from memory
                        break;
                    case 3:  // Store byte 3 of register to byte 0 of memory
                        newValue = ((gpr[rt] & 0xFF000000) >> 24) | // Reg byte 3 to mem byte 0
                                  (oldValue & 0xFFFFFF00);         // Keep bytes 1-3 from memory
                        break;
                }
                
                // Write back to memory
                memoryWrite(alignedAddress, newValue);
            }
            break;
            
        case 46:  // SWR - Store Word Right
            {
                // Get the aligned address
                uint32_t alignedAddress = address & ~0x3;
                
                // Read the current memory value
                uint32_t oldValue = memoryRead(alignedAddress);
                uint32_t newValue;
                
                // Determine which bytes to store based on the address
                switch (address & 0x3) {
                    case 0:  // Store byte 0 of register to byte 0 of memory
                        newValue = (gpr[rt] & 0x000000FF) |        // Reg byte 0 to mem byte 0
                                  (oldValue & 0xFFFFFF00);         // Keep bytes 1-3 from memory
                        break;
                    case 1:  // Store bytes 0-1 of register to bytes 1-2 of memory
                        newValue = ((gpr[rt] & 0x0000FFFF) << 8) | // Reg bytes 0-1 to mem bytes 1-2
                                  (oldValue & 0xFF0000FF);         // Keep bytes 0 and 3 from memory
                        break;
                    case 2:  // Store bytes 0-2 of register to bytes 2-4 of memory
                        newValue = ((gpr[rt] & 0x00FFFFFF) << 16) | // Reg bytes 0-2 to mem bytes 2-4
                                  (oldValue & 0x0000FFFF);         // Keep bytes 0-1 from memory
                        break;
                    case 3:  // Aligned - store the entire word
                        newValue = gpr[rt];
                        break;
                }
                
                // Write back to memory
                memoryWrite(alignedAddress, newValue);
            }
            break;
    }
}

void CPU::executeSpecial(uint32_t instruction) {
    // For SPECIAL instructions, the function is defined by the lowest 6 bits
    uint32_t function = instruction & 0x3F;
    uint32_t rs = (instruction >> 21) & 0x1F;
    uint32_t rt = (instruction >> 16) & 0x1F;
    uint32_t rd = (instruction >> 11) & 0x1F;
    uint32_t shamt = (instruction >> 6) & 0x1F;
    
    uint32_t result = 0;
    
    switch (function) {
        case 0x00:  // SLL - Shift Left Logical
            result = gpr[rt] << shamt;
            break;
            
        case 0x02:  // SRL - Shift Right Logical
            result = gpr[rt] >> shamt;
            break;
            
        case 0x03:  // SRA - Shift Right Arithmetic
            result = static_cast<uint32_t>(static_cast<int32_t>(gpr[rt]) >> shamt);
            break;
            
        case 0x04:  // SLLV - Shift Left Logical Variable
            result = gpr[rt] << (gpr[rs] & 0x1F);
            break;
            
        case 0x06:  // SRLV - Shift Right Logical Variable
            result = gpr[rt] >> (gpr[rs] & 0x1F);
            break;
            
        case 0x07:  // SRAV - Shift Right Arithmetic Variable
            result = static_cast<uint32_t>(static_cast<int32_t>(gpr[rt]) >> (gpr[rs] & 0x1F));
            break;
            
        case 0x08:  // JR - Jump Register
            delaySlot = true;
            nextPc = gpr[rs];
            return;  // No register write
            
        case 0x09:  // JALR - Jump and Link Register
            delaySlot = true;
            nextPc = gpr[rs];
            result = pc + 4;  // Fixed: Save return address (pc + 4, which is original_pc + 8)
            break;

        case 0x10:  // MFHI - Move From HI
            result = hi;
            break;
                    
        case 0x11:  // MTHI - Move To HI
            hi = gpr[rs];
            return;  // No register write
                    
        case 0x12:  // MFLO - Move From LO
            result = lo;
            break;
                    
        case 0x13:  // MTLO - Move To LO
            lo = gpr[rs];
            return;  // No register write

        case 0x18:  // MULT - Multiply
            {
                int64_t result = static_cast<int64_t>(static_cast<int32_t>(gpr[rs])) * 
                                static_cast<int64_t>(static_cast<int32_t>(gpr[rt]));
                lo = static_cast<uint32_t>(result & 0xFFFFFFFF);
                hi = static_cast<uint32_t>((result >> 32) & 0xFFFFFFFF);
            }
            return;  // No register write
                    
        case 0x19:  // MULTU - Multiply Unsigned
            {
                uint64_t result = static_cast<uint64_t>(gpr[rs]) * static_cast<uint64_t>(gpr[rt]);
                lo = static_cast<uint32_t>(result & 0xFFFFFFFF);
                hi = static_cast<uint32_t>((result >> 32) & 0xFFFFFFFF);
            }
            return;  // No register write

        case 0x1A:  // DIV - Divide
        {
            int32_t numerator = static_cast<int32_t>(gpr[rs]);
            int32_t denominator = static_cast<int32_t>(gpr[rt]);
            
            if (denominator == 0) {
                // Division by zero
                lo = (numerator >= 0) ? 0xFFFFFFFF : 1;
                hi = static_cast<uint32_t>(numerator);
            } else if (numerator == 0x80000000 && denominator == -1) {
                // Overflow case
                lo = 0x80000000;
                hi = 0;
            } else {
                lo = static_cast<uint32_t>(numerator / denominator);
                hi = static_cast<uint32_t>(numerator % denominator);
            }
        }
        return;  // No register write
                    
        case 0x1B:  // DIVU - Divide Unsigned
            {
                uint32_t numerator = gpr[rs];
                uint32_t denominator = gpr[rt];
                
                if (denominator == 0) {
                    // Division by zero
                    lo = 0xFFFFFFFF;
                    hi = numerator;
                } else {
                    lo = numerator / denominator;
                    hi = numerator % denominator;
                }
            }
            return;  // No register write
            
        case 0x20:  // ADD - Add (with overflow)
            {
                int32_t rs_signed = static_cast<int32_t>(gpr[rs]);
                int32_t rt_signed = static_cast<int32_t>(gpr[rt]);
                int32_t sum = rs_signed + rt_signed;
                
                // Check for overflow
                bool overflow = ((rs_signed >= 0 && rt_signed >= 0 && sum < 0) || 
                                (rs_signed < 0 && rt_signed < 0 && sum >= 0));
                
                if (overflow) {
                    triggerException(EXCEPTION_OVERFLOW, 0);
                    return;
                }
                
                result = static_cast<uint32_t>(sum);
            }
            break;
            
        case 0x21:  // ADDU - Add Unsigned (no overflow)
            result = gpr[rs] + gpr[rt];
            break;
            
        case 0x22:  // SUB - Subtract (with overflow)
            {
                int32_t rs_signed = static_cast<int32_t>(gpr[rs]);
                int32_t rt_signed = static_cast<int32_t>(gpr[rt]);
                int32_t diff = rs_signed - rt_signed;
                
                // Check for overflow
                bool overflow = ((rs_signed >= 0 && rt_signed < 0 && diff < 0) || 
                                (rs_signed < 0 && rt_signed >= 0 && diff >= 0));
                
                if (overflow) {
                    triggerException(EXCEPTION_OVERFLOW, 0);
                    return;
                }
                
                result = static_cast<uint32_t>(diff);
            }
            break;
            
        case 0x23:  // SUBU - Subtract Unsigned (no overflow)
            result = gpr[rs] - gpr[rt];
            break;
            
        case 0x24:  // AND
            result = gpr[rs] & gpr[rt];
            break;
            
        case 0x25:  // OR
            result = gpr[rs] | gpr[rt];
            break;
            
        case 0x26:  // XOR
            result = gpr[rs] ^ gpr[rt];
            break;
            
        case 0x27:  // NOR
            result = ~(gpr[rs] | gpr[rt]);
            break;
            
        case 0x2A:  // SLT - Set Less Than
            result = (static_cast<int32_t>(gpr[rs]) < static_cast<int32_t>(gpr[rt])) ? 1 : 0;
            break;
            
        case 0x2B:  // SLTU - Set Less Than Unsigned
            result = (gpr[rs] < gpr[rt]) ? 1 : 0;
            break;
            
        case 0x0C:  // SYSCALL
            triggerException(EXCEPTION_SYSCALL, 0);
            return;  // No register write
            
        case 0x0D:  // BREAK
            triggerException(EXCEPTION_BREAKPOINT, 0);
            return;  // No register write
            
        default:
            std::cerr << "Unimplemented SPECIAL function: " << std::hex << function << std::endl;
            triggerException(EXCEPTION_RESERVED_INSTRUCTION, 0);
            return;
    }
    
    // Write result to destination register (if not a JR instruction)
    setRegister(static_cast<Register>(rd), result);
}

void CPU::executeRegImm(uint32_t instruction) {
    uint32_t rs = (instruction >> 21) & 0x1F;
    uint32_t rt = (instruction >> 16) & 0x1F;
    int16_t offset = static_cast<int16_t>(instruction & 0xFFFF);
    uint32_t targetAddress = pc + (offset << 2);
    
    switch (rt) {
        case 0:  // BLTZ - Branch on Less Than Zero
            if (static_cast<int32_t>(gpr[rs]) < 0) {
                delaySlot = true;
                nextPc = targetAddress;
            }
            break;
        case 1:  // BGEZ - Branch on Greater Than or Equal to Zero
            if (static_cast<int32_t>(gpr[rs]) >= 0) {
                delaySlot = true;
                nextPc = targetAddress;
            }
            break;
        case 0x10: // BLTZAL - Branch on Less Than Zero And Link
            // Store return address in r31 (ra) regardless of branch condition
            gpr[31] = pc + 4;  // Fixed: Return address should be pc + 4 (which is original_pc + 8)
            if (static_cast<int32_t>(gpr[rs]) < 0) {
                // Set up the branch
                delaySlot = true;
                nextPc = targetAddress;
            }
            break;
        case 0x11: // BGEZAL - Branch on Greater Than or Equal to Zero And Link
            // Store return address in r31 (ra) regardless of branch condition
            gpr[31] = pc + 4;  // Fixed: Return address should be pc + 4 (which is original_pc + 8)
            if (static_cast<int32_t>(gpr[rs]) >= 0) {
                // Set up the branch
                delaySlot = true;
                nextPc = targetAddress;
            }
            break;
        default:
            std::cerr << "Unimplemented REGIMM instruction: " << std::hex << rt << std::endl;
            triggerException(EXCEPTION_RESERVED_INSTRUCTION, 0);
            break;
    }
}

void CPU::executeCoprocessor0(uint32_t instruction) {
    uint32_t rs = (instruction >> 21) & 0x1F;
    uint32_t rt = (instruction >> 16) & 0x1F;
    uint32_t rd = (instruction >> 11) & 0x1F;
    
    switch (rs) {
        case 0:  // MFC0 - Move from Coprocessor 0
            setRegister(static_cast<Register>(rt), getCP0Register(static_cast<CP0Register>(rd)));
            break;
        case 4:  // MTC0 - Move to Coprocessor 0
            setCP0Register(static_cast<CP0Register>(rd), gpr[rt]);
            break;
        case 16: // CO - Coprocessor Operation
            // For RFE, look at the lower bits of the instruction (function field)
            if ((instruction & 0x3F) == 0x10) {  // RFE - Return From Exception
                // Restore Status register by shifting the stack of bits:
                // Old → Previous, Previous → Current
                uint32_t status = cp0[CP0_SR];
                
                // Copy all bits except the mode bits (bits 5-0)
                uint32_t newStatus = status & 0xFFFFFFE0;
                
                // Extract the 2-bit mode values
                uint32_t oldMode = (status >> 4) & 0x3;     // Extract bits 5-4 (old mode)
                uint32_t prevMode = (status >> 2) & 0x3;    // Extract bits 3-2 (previous mode)
                
                // Set the new bit values:
                // - Bits 1-0 (current mode) = bits 3-2 (previous mode)
                // - Bits 3-2 (previous mode) = bits 5-4 (old mode)
                // - Bits 5-4 (old mode) remain unchanged
                newStatus |= (prevMode & 0x3);             // Current mode gets previous mode
                newStatus |= ((oldMode & 0x3) << 2);       // Previous mode gets old mode
                newStatus |= ((oldMode & 0x3) << 4);       // Old mode bits stay the same
                
                // Store the updated status register
                cp0[CP0_SR] = newStatus;
            } else {
                std::cerr << "Unimplemented COP0 function: " << std::hex << (instruction & 0x3F) << std::endl;
                triggerException(EXCEPTION_RESERVED_INSTRUCTION, 0);
            }
            break;
        default:
            std::cerr << "Unimplemented COP0 instruction: " << std::hex << rs << std::endl;
            triggerException(EXCEPTION_RESERVED_INSTRUCTION, 0);
            break;
    }
}

void CPU::executeCoprocessor2(uint32_t instruction) {
    uint32_t rs = (instruction >> 21) & 0x1F;  // Operation code
    uint32_t rt = (instruction >> 16) & 0x1F;  // CPU register
    uint32_t rd = (instruction >> 11) & 0x1F;  // GTE register/command
    
    switch (rs) {
        case 0x00:  // MFC2 - Move From Coprocessor 2
            // Should transfer data from GTE data register to CPU register
            std::cerr << "GTE: MFC2 not implemented (reg " << rd << " to " << REG_NAMES[rt] << ")" << std::endl;
            setRegister(static_cast<Register>(rt), 0);  // Return 0 for now
            break;
            
        case 0x02:  // CFC2 - Move Control From Coprocessor 2
            // Should transfer data from GTE control register to CPU register
            std::cerr << "GTE: CFC2 not implemented (reg " << rd << " to " << REG_NAMES[rt] << ")" << std::endl;
            setRegister(static_cast<Register>(rt), 0);  // Return 0 for now
            break;
            
        case 0x04:  // MTC2 - Move To Coprocessor 2
            // Should transfer data from CPU register to GTE data register
            std::cerr << "GTE: MTC2 not implemented (" << REG_NAMES[rt] << " to reg " << rd << ")" << std::endl;
            break;
            
        case 0x06:  // CTC2 - Move Control To Coprocessor 2
            // Should transfer data from CPU register to GTE control register
            std::cerr << "GTE: CTC2 not implemented (" << REG_NAMES[rt] << " to reg " << rd << ")" << std::endl;
            break;
            
        case 0x08:  // COP2 command - Primary GTE operation
            {
                uint32_t command = (instruction & 0x1FFFFFF);
                std::cerr << "GTE: Command 0x" << std::hex << command << " not implemented" << std::endl;
            }
            break;
            
        default:
            std::cerr << "Unimplemented COP2 instruction: rs=" << std::hex << rs << std::endl;
            triggerException(EXCEPTION_COPROCESSOR_UNUSABLE, 0);
            break;
    }
}

std::string CPU::disassembleInstruction(uint32_t instruction, uint32_t pc) {
    // TODO: Implement disassembler
    return "Not implemented";
}

} // namespace PSX