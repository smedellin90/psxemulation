#include "cpu_core.h"
#include <cstdint>
#include <iostream>
#include <map>
#include <format>
#include <functional>
#include <string>
#include <exception>
#include <cstddef>
#include <sys/errno.h>
#include <vector>

namespace PSXTest {

// Simple memory simulation for testing
class TestMemory {
private:
    std::map<uint32_t, uint32_t> contents;

public:
    uint32_t read(uint32_t address) {
        // Align to word boundary
        address &= ~0x3;
        
        auto it = contents.find(address);
        if (it != contents.end()) {
            return it->second;
        }

        // Return 0 for uninitialized memory
        return 0;
    }

    void write(uint32_t address, uint32_t value) {
        // Align to word boundary
        address &= ~0x3;
        contents[address] = value;
    }
    
    void loadProgram(uint32_t baseAddress, const std::vector<uint32_t>& instructions) {
        for (size_t i = 0; i < instructions.size(); i++) {
            write(baseAddress + i * 4, instructions[i]);
        }
    }
};

// Simple test framework
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

// Create a CPU with memory setup
void setupCPUWithMemory(PSX::CPU& cpu, TestMemory& mem) {
    cpu.setMemoryCallbacks(
        [&mem](uint32_t address) { return mem.read(address); },
        [&mem](uint32_t address, uint32_t value) { mem.write(address, value); }
    );
    cpu.reset();
}

void resetPCOnly(PSX::CPU& cpu) {
    // Reset only the PC to ensure the next instruction executes correctly
    cpu.setPC(0xBFC00000);
}

    // Instruction encoding helpers.
namespace Encode {
    // R-type instruction encoding
    uint32_t R(uint32_t rs, uint32_t rt, uint32_t rd, uint32_t shamt, uint32_t funct) {
        return (rs << 21) | (rt << 16) | (rd << 11) | (shamt << 6) | funct;
    }
    
    // I-type instruction encoding
    uint32_t I(uint32_t opcode, uint32_t rs, uint32_t rt, int16_t imm) {
        return (opcode << 26) | (rs << 21) | (rt << 16) | (uint16_t)imm;
    }
    
    // J-type instruction encoding
    uint32_t J(uint32_t opcode, uint32_t address) {
        return (opcode << 26) | (address & 0x3FFFFFF);
    }
    
// Common instructions
namespace R_Type {
    uint32_t MULT(uint32_t rs, uint32_t rt) {
        return R(rs, rt, 0, 0, 0x18);
    }
    
    uint32_t MULTU(uint32_t rs, uint32_t rt) {
        return R(rs, rt, 0, 0, 0x19);
    }
    
    uint32_t DIV(uint32_t rs, uint32_t rt) {
        return R(rs, rt, 0, 0, 0x1A);
    }
    
    uint32_t DIVU(uint32_t rs, uint32_t rt) {
        return R(rs, rt, 0, 0, 0x1B);
    }
    
    uint32_t MFHI(uint32_t rd) {
        return R(0, 0, rd, 0, 0x10);
    }
    
    uint32_t MFLO(uint32_t rd) {
        return R(0, 0, rd, 0, 0x12);
    }
    
    uint32_t MTHI(uint32_t rs) {
        return R(rs, 0, 0, 0, 0x11);
    }
    
    uint32_t MTLO(uint32_t rs) {
        return R(rs, 0, 0, 0, 0x13);
    }

    uint32_t SLL(uint32_t rd, uint32_t rt, uint32_t shamt) {
        return R(0, rt, rd, shamt, 0);
    }
    
    uint32_t SRL(uint32_t rd, uint32_t rt, uint32_t shamt) {
        return R(0, rt, rd, shamt, 2);
    }
    
    uint32_t SRA(uint32_t rd, uint32_t rt, uint32_t shamt) {
        return R(0, rt, rd, shamt, 3);
    }
    
    uint32_t SLLV(uint32_t rd, uint32_t rt, uint32_t rs) {
        return R(rs, rt, rd, 0, 4);
    }
    
    uint32_t SRLV(uint32_t rd, uint32_t rt, uint32_t rs) {
        return R(rs, rt, rd, 0, 6);
    }
    
    uint32_t SRAV(uint32_t rd, uint32_t rt, uint32_t rs) {
        return R(rs, rt, rd, 0, 7);
    }
    
    uint32_t JR(uint32_t rs) {
        return R(rs, 0, 0, 0, 8);
    }
    
    uint32_t JALR(uint32_t rd, uint32_t rs) {
        return R(rs, 0, rd, 0, 9);
    }
    
    uint32_t ADD(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x20);
    }
    
    uint32_t ADDU(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x21);
    }
    
    uint32_t SUB(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x22);
    }
    
    uint32_t SUBU(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x23);
    }
    
    uint32_t AND(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x24);
    }
    
    uint32_t OR(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x25);
    }
    
    uint32_t XOR(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x26);
    }
    
    uint32_t NOR(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x27);
    }
    
    uint32_t SLT(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x2A);
    }
    
    uint32_t SLTU(uint32_t rd, uint32_t rs, uint32_t rt) {
        return R(rs, rt, rd, 0, 0x2B);
    }
    
    uint32_t SYSCALL() {
        return R(0, 0, 0, 0, 0x0C);
    }
    
    uint32_t BREAK() {
        return R(0, 0, 0, 0, 0x0D);
    }
    
    // COP0 RFE (Return From Exception) instruction
    uint32_t RFE() {
        return (0x10 << 26) | (0x10 << 21) | 0x10; // COP0 (opcode 0x10), CO=1 (rs=0x10), function=0x10
    }
}

namespace I_Type {
    uint32_t ADDI(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(8, rs, rt, imm);
    }
    
    uint32_t ADDIU(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(9, rs, rt, imm);
    }
    
    uint32_t SLTI(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(0x0A, rs, rt, imm);
    }
    
    uint32_t SLTIU(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(0x0B, rs, rt, imm);
    }
    
    uint32_t ANDI(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(0xC, rs, rt, imm);
    }
    
    uint32_t ORI(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(0xD, rs, rt, imm);
    }
    
    uint32_t XORI(uint32_t rt, uint32_t rs, int16_t imm) {
        return I(0xE, rs, rt, imm);
    }
    
    uint32_t LUI(uint32_t rt, int16_t imm) {
        return I(0xF, 0, rt, imm);
    }
    
    uint32_t BEQ(uint32_t rs, uint32_t rt, int16_t offset) {
        return I(4, rs, rt, offset);
    }
    
    uint32_t BNE(uint32_t rs, uint32_t rt, int16_t offset) {
        return I(5, rs, rt, offset);
    }

    uint32_t BLTZ(uint32_t rs, int16_t offset) {
        return I(1, rs, 0, offset);
    }
    
    uint32_t BGEZ(uint32_t rs, int16_t offset) {
        return I(1, rs, 1, offset);
    }
    
    uint32_t BLTZAL(uint32_t rs, int16_t offset) {
        return I(1, rs, 0x10, offset);
    }
    
    uint32_t BGEZAL(uint32_t rs, int16_t offset) {
        return I(1, rs, 0x11, offset);
    }

    uint32_t BLEZ(uint32_t rs, int16_t offset) {
        return I(6, rs, 0, offset);
    }
    
    uint32_t BGTZ(uint32_t rs, int16_t offset) {
        return I(7, rs, 0, offset);
    }
    
    uint32_t BC0F(int16_t offset) {
        return I(0x10, 8, 0, offset);
    }
    
    uint32_t BC0T(int16_t offset) {
        return I(0x10, 8, 1, offset);
    }
    
    uint32_t BC1F(int16_t offset) {
        return I(0x11, 8, 0, offset);
    }
    
    uint32_t BC1T(int16_t offset) {
        return I(0x11, 8, 1, offset);
    }
    
    uint32_t BC2F(int16_t offset) {
        return I(0x12, 8, 0, offset);
    }
    
    uint32_t BC2T(int16_t offset) {
        return I(0x12, 8, 1, offset);
    } 

    uint32_t LB(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x20, rs, rt, offset);
    }
    
    uint32_t LBU(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x24, rs, rt, offset);
    }
    
    uint32_t LH(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x21, rs, rt, offset);
    }
    
    uint32_t LHU(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x25, rs, rt, offset);
    }
    
    uint32_t LWL(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x22, rs, rt, offset);
    }
    
    uint32_t LW(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x23, rs, rt, offset);
    }
    
    uint32_t LWR(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x26, rs, rt, offset);
    }
    
    uint32_t SB(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x28, rs, rt, offset);
    }
    
    uint32_t SH(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x29, rs, rt, offset);
    }
    
    uint32_t SWL(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x2A, rs, rt, offset);
    }
    
    uint32_t SW(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x2B, rs, rt, offset);
    }
    
    uint32_t SWR(uint32_t rt, uint32_t rs, int16_t offset) {
        return I(0x2E, rs, rt, offset);
    }
}

namespace J_Type {
    uint32_t J(uint32_t target) {
        return Encode::J(2, target >> 2);
    }
    
    uint32_t JAL(uint32_t target) {
        return Encode::J(3, target >> 2);
    }
}
}

// Run tests
void runTests() {
    TestRunner runner;
    
    // Test 1: Basic CPU Initialization
    runner.addTest("Basic CPU Initialization", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        runner.check(cpu.getPC() == 0xBFC00000, "PC should be initialized to BIOS entry point");
        runner.check(cpu.getRegister(PSX::CPU::R0) == 0, "R0 should be zero");
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_PRID) == 0x00000002, "PRID should be set correctly");
    });
    
    // Test 2: R0 hardwired to zero
    runner.addTest("Register R0 Always Zero", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::R0, 0xDEADBEEF);
        runner.check(cpu.getRegister(PSX::CPU::R0) == 0, "R0 should remain zero after write attempt");
    });
    
    // Test 3: Basic ALU operations
    runner.addTest("ALU Operations - ADDIU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // addiu $v0, $zero, 42 (0x2A)
        mem.write(0xBFC00000, Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::R0, 42));
        
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::V0) == 42, "ADDIU should add immediate to register");
        runner.check(cpu.getPC() == 0xBFC00004, "PC should advance by 4");
    });

    // Adding more test cases for my slow human brain to catch up.
    runner.addTest("ALU Operations - ADDIU v2", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // addiu $v1, $zero, 159 (0x9F)
        mem.write(0xBFC00000, Encode::I_Type::ADDIU(PSX::CPU::V1, PSX::CPU::R0, 159));
    
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::V1) == 159, "ADDIU should add immediate to register");
        runner.check(cpu.getPC() == 0xBFC00004, "PC should advance by 4");
    });

    // Let's chain two add operations.
    // r0 + 159 => v1
    // v1 + 31  => v1
    runner.addTest("ALU Operations - ADDIU v3", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // addiu $v1, $r0,   159 (0x9F)
        mem.write(0xBFC00000, Encode::I_Type::ADDIU(PSX::CPU::V1, PSX::CPU::R0, 159));

        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::V1) == 159, "ADDIU should add immediate to register");
        runner.check(cpu.getPC() == 0xBFC00004, "PC should advance by 4");

        // addiu $v1, $v1,   32  (0x20)
        mem.write(0xBFC00004, Encode::I_Type::ADDIU(PSX::CPU::V1, PSX::CPU::V1, 32));

        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::V1) == 191, "ADDIU should add immediate to register");
        runner.check(cpu.getPC() == 0xBFC00008, "PC should advance by 4");
    });
    
    // Test 4: ALU operations with negative immediate
    runner.addTest("ALU Operations - ADDIU with negative immediate", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // addiu $v0, $zero, -10 (0xFFF6)
        mem.write(0xBFC00000, Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::R0, -10));

        cpu.executeInstruction();

        runner.check(cpu.getRegister(PSX::CPU::V0) == 0xFFFFFFF6, "ADDIU should sign-extend negative immediate");
    });
    
    // Test 5: ALU operations - ADDU
    runner.addTest("ALU Operations - ADDU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::V0, 25);
        cpu.setRegister(PSX::CPU::V1, 17);
        
        // addu $a0, $v0, $v1
        // mem.write(0xBFC00000, 0x00432021);
        mem.write(0xBFC00000, Encode::R_Type::ADDU(PSX::CPU::A0, PSX::CPU::V0, PSX::CPU::V1));
        
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::A0) == 42, "ADDU should add two registers");
    });
    
    // Test 6: Basic Jump
    runner.addTest("Jump Instructions - J", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);

        uint32_t expected = 0xB0100000;
        
        // j 0xB0100000
        mem.write(0xBFC00000, Encode::J_Type::J(0xB0100000));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute jump
        runner.check(cpu.getPC() == 0xBFC00004, "PC should be in delay slot after jump");
        
        cpu.executeInstruction();  // Execute delay slot
        runner.check(cpu.getPC() == expected, std::format("PC should be at jump target after delay slot {:X} != {:X}", cpu.getPC(), expected));
    });
    
    // Test 7: Jump and Link
    runner.addTest("Jump Instructions - JAL", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);

        uint32_t expected = 0xB0100000;
        
        // jal 0xB0100000
        mem.write(0xBFC00000, Encode::J_Type::JAL(0xB0100000));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute jal
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, "RA should be set to return address");
        
        cpu.executeInstruction();  // Execute delay slot
        runner.check(cpu.getPC() == expected, std::format("PC should be at jump target after delay slot {:X} != {:X}", cpu.getPC(), expected));
    });
    
    // Test 8: Branch Equal
    runner.addTest("Branch Instructions - BEQ", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::T0, 5);
        cpu.setRegister(PSX::CPU::T1, 5);
        
        // beq $t0, $t1, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BEQ(PSX::CPU::T0, PSX::CPU::T1, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute beq
        runner.check(cpu.getPC() == 0xBFC00004, "PC should be in delay slot");
        
        cpu.executeInstruction();  // Execute delay slot
        runner.check(cpu.getPC() == 0xBFC00044, std::format("PC should be at branch target after delay slot ({:X} != {:X})", cpu.getPC(), 0xBFC00044));
    });
    
    // Test 9: Branch Not Equal
    runner.addTest("Branch Instructions - BNE", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::T0, 5);
        cpu.setRegister(PSX::CPU::T1, 10);
        
        // bne $t0, $t1, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BNE(PSX::CPU::T0, PSX::CPU::T1, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bne
        runner.check(cpu.getPC() == 0xBFC00004, "PC should be in delay slot");
        
        cpu.executeInstruction();  // Execute delay slot
        runner.check(cpu.getPC() == 0xBFC00044, "PC should be at branch target after delay slot");
    });

    // Branch Not Equal, but it is equal. 
    runner.addTest("Branch Instructions - BNE but it is equal", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::T0, 5);
        cpu.setRegister(PSX::CPU::T1, 5);
        
        // bne $t0, $t1, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BNE(PSX::CPU::T0, PSX::CPU::T1, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bne
        runner.check(cpu.getPC() == 0xBFC00004, "PC should be in delay slot");
        
        cpu.executeInstruction();  // Continue execution as normal
        runner.check(cpu.getPC() == 0xBFC00008, "PC should be executing without branching");
    });
    
    // Test 10: Memory Access - LW/SW
    runner.addTest("Memory Operations - LW/SW", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::S0, 0x80000000);
        cpu.setRegister(PSX::CPU::T0, 0x12345678);
        
        // sw $t0, 0($s0) - Store word
        mem.write(0xBFC00000, Encode::I_Type::SW(PSX::CPU::T0, PSX::CPU::S0, 0));
        
        cpu.executeInstruction();
        runner.check(mem.read(0x80000000) == 0x12345678, "SW should store word to memory");
        
        // lw $t1, 0($s0) - Load word
        mem.write(0xBFC00004, Encode::I_Type::LW(PSX::CPU::T1, PSX::CPU::S0, 0));
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T1) == 0x12345678, "LW should load word from memory");
    });
    
    // Test 11: Memory Access - LB (Load Byte)
    runner.addTest("Memory Operations - LB", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up memory with a word containing 0xFFA5B2C3
        mem.write(0x80000000, 0xFFA5B2C3);
        
        cpu.setRegister(PSX::CPU::S0, 0x80000000);
        
        // lb $t0, 0($s0) - Load byte (should get 0xC3, sign-extended)
        mem.write(0xBFC00000, Encode::I_Type::LB(PSX::CPU::T0, PSX::CPU::S0, 0));
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0xFFFFFFC3, 
                    "LB should load and sign-extend the byte");

        // lb $t0, 1($s0) - Load byte (should get 0xB2, sign-extended)
        mem.write(0xBFC00004, Encode::I_Type::LB(PSX::CPU::T0, PSX::CPU::S0, 1));
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0xFFFFFFB2, 
                    "LB should load and sign-extend the byte");
    });
    
    // Test 12: Shift Operations
    runner.addTest("Shift Operations", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::T0, 0x00000010);
        
        // sll $t1, $t0, 4 (logical shift left)
        mem.write(0xBFC00000, Encode::R_Type::SLL(PSX::CPU::T1, PSX::CPU::T0, 4));
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T1) == 0x00000100, 
                    "SLL should shift left by specified amount");
        
        // srl $t2, $t0, 2 (logical shift right)
        mem.write(0xBFC00004, Encode::R_Type::SRL(PSX::CPU::T2, PSX::CPU::T0, 2));
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0x00000004, 
                    "SRL should shift right by specified amount");
        
        cpu.setRegister(PSX::CPU::T0, 0x80000010);
        
        // sra $t3, $t0, 4 (arithmetic shift right)
        mem.write(0xBFC00008, Encode::R_Type::SRA(PSX::CPU::T3, PSX::CPU::T0, 4));
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0xF8000001, 
                    "SRA should arithmetic shift right, preserving sign bit");
    });
    
    // Test 13: Overflow Exception in ADD
    runner.addTest("Exception - Overflow in ADD", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::T0, 0x7FFFFFFF);  // Max positive int
        cpu.setRegister(PSX::CPU::T1, 0x00000001);  // 1
        
        // add $t2, $t0, $t1 (should overflow)
        mem.write(0xBFC00000, Encode::R_Type::ADD(PSX::CPU::T2, PSX::CPU::T0, PSX::CPU::T1));
        
        cpu.executeInstruction();
        
        runner.check(cpu.getPC() == 0xBFC00180, 
                    "Overflow exception should set PC to exception vector");
        
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x7C) == 
                    (PSX::CPU::EXCEPTION_OVERFLOW << 2), 
                    "Cause register should indicate overflow exception");
    });
    
    // Test 14: Coprocessor 0 Operations
    runner.addTest("Coprocessor 0 Operations - MTC0/MFC0", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::T0, 0xAAAAAAAA);
        
        // mtc0 $t0, $12 (Status register)
        mem.write(0xBFC00000, 0x40886000);
        
        cpu.executeInstruction();
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_SR) == 0xAAAAAAAA, 
                    "MTC0 should move value to CP0 register");
        
        // mfc0 $t1, $12 (Status register)
        mem.write(0xBFC00004, 0x40096000);
        
        cpu.executeInstruction();
        runner.check(cpu.getRegister(PSX::CPU::T1) == 0xAAAAAAAA, 
                    "MFC0 should move value from CP0 register");
    });
    
    // Test 15: Complex program - Fibonacci calculation
    runner.addTest("Complex Program - Fibonacci", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Load a program that calculates Fibonacci(10)
        std::vector<uint32_t> program = {
            Encode::I_Type::ADDIU(PSX::CPU::A0, PSX::CPU::R0, 10),  // li $a0, 10       # n = 10 - 0
            Encode::I_Type::ADDIU(PSX::CPU::V0, PSX::CPU::R0, 0),   // li $v0, 0        # fib0 = 0 - 4
            Encode::I_Type::ADDIU(PSX::CPU::V1, PSX::CPU::R0, 1),   // li $v1, 1        # fib1 = 1 - 8
          /*loop:*/
            Encode::R_Type::ADD(PSX::CPU::T0, PSX::CPU::V0, PSX::CPU::V1),  // add $t0, $v0, $v1  # t0 = fib0 + fib1 - C
            Encode::R_Type::ADDU(PSX::CPU::V0, PSX::CPU::R0, PSX::CPU::V1), // move $v0, $v1      # fib0 = fib1 - 10
            Encode::R_Type::ADDU(PSX::CPU::V1, PSX::CPU::R0, PSX::CPU::T0), // move $v1, $t0      # fib1 = t0 - 14
            Encode::I_Type::ADDI(PSX::CPU::A0, PSX::CPU::A0, -1),   // addi $a0, $a0, -1  # n-- - 18
            Encode::I_Type::BNE(PSX::CPU::A0, PSX::CPU::R0, -5),    // bne $a0, $zero, loop - 1C
            0x00000000,                                             // nop - 20
          /*end:*/
        };
        
        mem.loadProgram(0xBFC00000, program);
        
        std::cout << "=== FIBONACCI TEST DEBUG ===" << std::endl;
        // Execute until we reach the branch at the end, or hit 100 instructions (safety)
        for (int i = 0; i < 100; i++) {
            uint32_t pc_before = cpu.getPC();
            uint32_t instr = mem.read(pc_before);
            
            // Debug if this is a branch instruction
            if ((instr >> 26) == 5) {  // BNE
                uint32_t rs = (instr >> 21) & 0x1F;
                uint32_t rt = (instr >> 16) & 0x1F;
                int16_t offset = static_cast<int16_t>(instr & 0xFFFF);
                bool notEqual = (cpu.getRegister(static_cast<PSX::CPU::Register>(rs)) != 
                                cpu.getRegister(static_cast<PSX::CPU::Register>(rt)));
                std::cout << std::format("BNE: rs=${:d}={:d}, rt=${:d}={:d}, notEqual={}, offset={:d}, target=0x{:08X}\n", 
                    rs, cpu.getRegister(static_cast<PSX::CPU::Register>(rs)), 
                    rt, cpu.getRegister(static_cast<PSX::CPU::Register>(rt)), 
                    notEqual ? "true" : "false",
                    offset, pc_before + 4 + (offset << 2));
            }
            
            cpu.executeInstruction();
            
            std::cout << std::format("Step {:2d}: PC=0x{:08X} -> 0x{:08X}, Instr=0x{:08X}, a0={:d}, v0={:d}, v1={:d}, t0={:d}\n",
                i, pc_before, cpu.getPC(), instr,
                cpu.getRegister(PSX::CPU::A0),
                cpu.getRegister(PSX::CPU::V0),
                cpu.getRegister(PSX::CPU::V1),
                cpu.getRegister(PSX::CPU::T0));
            
            // If we've reached the end instruction, we're done
            if (cpu.getPC() == 0xBFC00028) {
                std::cout << "Reached end instruction" << std::endl;
                break;
            }
            
            // Safety check for infinite loops
            if (i == 99) {
                std::cout << "Maximum iteration count reached, possible infinite loop" << std::endl;
            }
        }
        
        std::cout << "Final v0 = " << cpu.getRegister(PSX::CPU::V0) << " (expected 55)" << std::endl;
        std::cout << "==============================" << std::endl;
        
        runner.check(cpu.getRegister(PSX::CPU::V0) == 55, "Fibonacci(10) should equal 55");
    });
    
    // Test 16: Load/Store Exceptions - Unaligned Access
    runner.addTest("Exception - Unaligned Access", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        cpu.setRegister(PSX::CPU::S0, 0x80000001);  // Unaligned address
        
        // lw $t0, 0($s0) - Load word from unaligned address
        mem.write(0xBFC00000, Encode::I_Type::LW(PSX::CPU::T0, PSX::CPU::S0, 0));
        
        cpu.executeInstruction();
        
        runner.check(cpu.getPC() == 0xBFC00180, 
                    "Address error exception should set PC to exception vector");
        
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x7C) == 
                    (PSX::CPU::EXCEPTION_ADDRESS_ERROR_LOAD << 2), 
                    "Cause register should indicate address error exception");
        
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_BADVADDR) == 0x80000001, 
                    "BadVAddr should contain the faulting address");
    });
    
    // Test 17: REGIMM Instructions - BGEZ/BLTZ
    runner.addTest("REGIMM Instructions - BGEZ/BLTZ", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test BGEZ
        cpu.setRegister(PSX::CPU::T0, 10);  // Positive value
        
        // bgez $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BGEZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bgez
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00044, 
                    std::format("BGEZ should branch when value is positive. expected 0xBFC00044, got {:X}", cpu.getPC()));
        
        // Test BLTZ
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, -10);  // Negative value
        
        // bltz $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BLTZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bltz
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00044, 
                    "BLTZ should branch when value is negative");
    });
    
    // Test 18: REGIMM Instructions - BGEZAL/BLTZAL
    runner.addTest("REGIMM Instructions - BGEZAL/BLTZAL", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test BGEZAL
        cpu.setRegister(PSX::CPU::T0, 10);  // Positive value
        
        // bgezal $t0, 0xFFFF
        mem.write(0xBFC00000, Encode::I_Type::BGEZAL(PSX::CPU::T0, 0xFFFF));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bgezal
        
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("BGEZAL should set return address. Expected 0xBFC00008, got {:X}", cpu.getRegister(PSX::CPU::RA)));
        
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00000, 
                    "BGEZAL should branch when value is positive");
    });

    // Test for comprehensive branch-and-link return address calculation
    runner.addTest("Branch-and-Link Return Address Calculation", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test 1: JAL instruction
        // jal 0xB0100000
        mem.write(0xBFC00000, Encode::J_Type::JAL(0xB0100000));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute JAL
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("JAL should set return address to PC+8. Expected 0xBFC00008, got {:X}", 
                                cpu.getRegister(PSX::CPU::RA)));
        
        // Reset for next test
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        
        // Test 2: JALR instruction
        cpu.setRegister(PSX::CPU::T0, 0xB0100000);  // Jump target
        
        // jalr $ra, $t0
        mem.write(0xBFC00000, Encode::R_Type::JALR(PSX::CPU::RA, PSX::CPU::T0));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute JALR
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("JALR should set return address to PC+8. Expected 0xBFC00008, got {:X}", 
                                cpu.getRegister(PSX::CPU::RA)));
        
        // Reset for next test
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        
        // Test 3: BGEZAL instruction
        cpu.setRegister(PSX::CPU::T0, 10);  // Positive value
        
        // bgezal $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BGEZAL(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute BGEZAL
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("BGEZAL should set return address to PC+8. Expected 0xBFC00008, got {:X}", 
                                cpu.getRegister(PSX::CPU::RA)));
        
        // Reset for next test
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        
        // Test 4: BLTZAL instruction
        cpu.setRegister(PSX::CPU::T0, -10);  // Negative value
        
        // bltzal $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BLTZAL(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute BLTZAL
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("BLTZAL should set return address to PC+8. Expected 0xBFC00008, got {:X}", 
                                cpu.getRegister(PSX::CPU::RA)));
        
        // Test 5: BGEZAL with branch not taken
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, -10);  // Negative value (branch not taken)
        
        // bgezal $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BGEZAL(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute BGEZAL
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("BGEZAL should set return address to PC+8 even when branch not taken. Expected 0xBFC00008, got {:X}", 
                                cpu.getRegister(PSX::CPU::RA)));
        
        // Test 6: BLTZAL with branch not taken
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, 10);  // Positive value (branch not taken)
        
        // bltzal $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BLTZAL(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute BLTZAL
        runner.check(cpu.getRegister(PSX::CPU::RA) == 0xBFC00008, 
                    std::format("BLTZAL should set return address to PC+8 even when branch not taken. Expected 0xBFC00008, got {:X}", 
                                cpu.getRegister(PSX::CPU::RA)));
    });

    // Test for Multiplication Operations
    runner.addTest("Multiplication Operations - MULT/MULTU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test MULT - Simple multiplication
        cpu.setRegister(PSX::CPU::T0, 5);
        cpu.setRegister(PSX::CPU::T1, 10);
        
        // mult $t0, $t1
        mem.write(0xBFC00000, Encode::R_Type::MULT(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2
        mem.write(0xBFC00004, Encode::R_Type::MFLO(PSX::CPU::T2));
        
        cpu.executeInstruction();  // Execute mult
        cpu.executeInstruction();  // Execute mflo
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 50, 
                    "MULT should multiply values and store in LO");
        
        // Test MULT - Negative values
        cpu.setRegister(PSX::CPU::T0, -5);
        cpu.setRegister(PSX::CPU::T1, 10);
        
        // mult $t0, $t1
        mem.write(0xBFC00008, Encode::R_Type::MULT(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2
        mem.write(0xBFC0000C, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3
        mem.write(0xBFC00010, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute mult
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0xFFFFFFCE, 
                    "MULT should handle negative values correctly (LO)");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0xFFFFFFFF, 
                    "MULT should sign-extend the result into HI");
        
        // Test MULTU - Large unsigned multiplication
        cpu.setRegister(PSX::CPU::T0, 0x10000000);  // 268,435,456
        cpu.setRegister(PSX::CPU::T1, 0x10);        // 16
        
        // multu $t0, $t1
        mem.write(0xBFC00014, Encode::R_Type::MULTU(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2
        mem.write(0xBFC00018, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3
        mem.write(0xBFC0001C, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute multu
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0, 
                    "MULTU should handle large values correctly (LO)");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0x1, 
                    "MULTU should handle large values correctly (HI)");
    });

    // Test for Division Operations
    runner.addTest("Division Operations - DIV/DIVU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test DIV - Simple division
        cpu.setRegister(PSX::CPU::T0, 100);
        cpu.setRegister(PSX::CPU::T1, 10);
        
        // div $t0, $t1
        mem.write(0xBFC00000, Encode::R_Type::DIV(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2 (quotient)
        mem.write(0xBFC00004, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3 (remainder)
        mem.write(0xBFC00008, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute div
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 10, 
                    "DIV should store quotient in LO");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0, 
                    "DIV should store remainder in HI");
        
        // Test DIV - Division with remainder
        cpu.setRegister(PSX::CPU::T0, 103);
        cpu.setRegister(PSX::CPU::T1, 10);
        
        // div $t0, $t1
        mem.write(0xBFC0000C, Encode::R_Type::DIV(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2 (quotient)
        mem.write(0xBFC00010, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3 (remainder)
        mem.write(0xBFC00014, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute div
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 10, 
                    "DIV should truncate division result");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 3, 
                    "DIV should calculate correct remainder");
        
        // Test DIV - Division by zero
        cpu.setRegister(PSX::CPU::T0, 100);
        cpu.setRegister(PSX::CPU::T1, 0);
        
        // div $t0, $t1
        mem.write(0xBFC00018, Encode::R_Type::DIV(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2 (quotient)
        mem.write(0xBFC0001C, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3 (remainder)
        mem.write(0xBFC00020, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute div
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0xFFFFFFFF, 
                    "DIV should handle division by zero correctly (LO)");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 100, 
                    "DIV should handle division by zero correctly (HI)");
        
        // Test DIV - Negative division
        cpu.setRegister(PSX::CPU::T0, -100);
        cpu.setRegister(PSX::CPU::T1, 10);
        
        // div $t0, $t1
        mem.write(0xBFC00024, Encode::R_Type::DIV(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2 (quotient)
        mem.write(0xBFC00028, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3 (remainder)
        mem.write(0xBFC0002C, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute div
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0xFFFFFFF6, 
                    "DIV should handle negative values correctly (LO)");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0, 
                    "DIV should handle negative values correctly (HI)");
        
        // Test DIVU - Unsigned division
        cpu.setRegister(PSX::CPU::T0, 0xFFFFFFFF);  // Max unsigned 32-bit
        cpu.setRegister(PSX::CPU::T1, 0x10);        // 16
        
        // divu $t0, $t1
        mem.write(0xBFC00030, Encode::R_Type::DIVU(PSX::CPU::T0, PSX::CPU::T1));
        // mflo $t2 (quotient)
        mem.write(0xBFC00034, Encode::R_Type::MFLO(PSX::CPU::T2));
        // mfhi $t3 (remainder)
        mem.write(0xBFC00038, Encode::R_Type::MFHI(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute divu
        cpu.executeInstruction();  // Execute mflo
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0x0FFFFFFF, 
                    "DIVU should handle large unsigned values correctly (LO)");
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0xF, 
                    "DIVU should handle large unsigned values correctly (HI)");
    });

    // Test for HI/LO Register Operations
    runner.addTest("HI/LO Register Operations - MTHI/MTLO/MFHI/MFLO", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test MTHI/MFHI
        cpu.setRegister(PSX::CPU::T0, 0xABCD1234);
        
        // mthi $t0
        mem.write(0xBFC00000, Encode::R_Type::MTHI(PSX::CPU::T0));
        // mfhi $t1
        mem.write(0xBFC00004, Encode::R_Type::MFHI(PSX::CPU::T1));
        
        cpu.executeInstruction();  // Execute mthi
        cpu.executeInstruction();  // Execute mfhi
        
        runner.check(cpu.getRegister(PSX::CPU::T1) == 0xABCD1234, 
                    "MTHI/MFHI should move values to/from HI register");
        
        // Test MTLO/MFLO
        cpu.setRegister(PSX::CPU::T2, 0x12345678);
        
        // mtlo $t2
        mem.write(0xBFC00008, Encode::R_Type::MTLO(PSX::CPU::T2));
        // mflo $t3
        mem.write(0xBFC0000C, Encode::R_Type::MFLO(PSX::CPU::T3));
        
        cpu.executeInstruction();  // Execute mtlo
        cpu.executeInstruction();  // Execute mflo
        
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0x12345678, 
                    "MTLO/MFLO should move values to/from LO register");
    });
    
    // Test for unaligned load/store instructions
    runner.addTest("Unaligned Memory Operations - LWL/LWR", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Initialize memory with known values - little endian
        // Memory at 0x80000000 will be 11 22 33 44 (in byte order)
        // Memory at 0x80000004 will be 55 66 77 88 (in byte order)
        mem.write(0x80000000, 0x44332211);
        mem.write(0x80000004, 0x88776655);
        
        std::cout << "Memory initialized for LWL/LWR tests" << std::endl;
        
        // ---- Test LWL with different alignments ----
        std::cout << "\n----- LWL TESTS -----" << std::endl;
        
        // Case 1: LWL at address 0x80000000 (aligned)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000000);
        uint32_t lwl_instruction = Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwl_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x44332211, 
                    "LWL aligned should load the entire word");
        
        // Case 2: LWL at address 0x80000001 (offset 1)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000001); // This sets address to 0x80000001 (offset 1)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        lwl_instruction = Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwl_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x332211DD, 
                    "LWL offset 1 should load bytes 1-3 from memory to positions 3-1");
        
        // Case 3: LWL at address 0x80000002 (offset 2)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000002); // This sets address to 0x80000002 (offset 2)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        lwl_instruction = Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwl_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x2211CCDD, 
                    "LWL offset 2 should load bytes 2-3 from memory to positions 3-2");
        
        // Case 4: LWL at address 0x80000003 (offset 3)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000003); // This sets address to 0x80000003 (offset 3)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        lwl_instruction = Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwl_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x11BBCCDD, 
                    "LWL offset 3 should load byte 3 from memory to position 3");
        
        // ---- Test LWR with different alignments ----
        std::cout << "\n----- LWR TESTS -----" << std::endl;
        
        // Case 1: LWR at address 0x80000000 (offset 0)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000000); // This sets address to 0x80000000 (offset 0)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        uint32_t lwr_instruction = Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwr_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0xAABBCC11, 
                    "LWR offset 0 should load byte 0 from memory to position 0");
        
        // Case 2: LWR at address 0x80000001 (offset 1)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000001); // This sets address to 0x80000001 (offset 1)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        lwr_instruction = Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwr_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0xAABB2211, 
                    "LWR offset 1 should load bytes 0-1 from memory to positions 0-1");
        
        // Case 3: LWR at address 0x80000002 (offset 2)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000002); // This sets address to 0x80000002 (offset 2)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        lwr_instruction = Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwr_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0xAA332211, 
                    "LWR offset 2 should load bytes 0-2 from memory to positions 0-2");
        
        // Case 4: LWR at address 0x80000003 (offset 3)
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);
        cpu.setRegister(PSX::CPU::S0, 0x80000003); // This sets address to 0x80000003 (offset 3)
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        lwr_instruction = Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0);
        mem.write(0xBFC00000, lwr_instruction);
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x44332211, 
                    "LWR offset 3 should load entire word from memory");
                    
        // Test combined LWL+LWR for unaligned word access
        std::cout << "\n----- COMBINED LWL+LWR TESTS -----" << std::endl;
        
        // Unaligned word at address 0x80000001
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);  // Initial value
        cpu.setRegister(PSX::CPU::S0, 0x80000001);  // Address
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        // Execute LWL first (loads bytes 1-3 into positions 3-1)
        mem.write(0xBFC00000, Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0));
        cpu.executeInstruction();
        
        // Now execute LWR to complete the unaligned word load
        resetPCOnly(cpu);
        uint32_t address = 0x80000001 + 3;  // End of the word (0x80000004)
        cpu.setRegister(PSX::CPU::S0, address);
        
        mem.write(0xBFC00000, Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0));
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x33221155, 
                    "Combined LWL+LWR should correctly load an unaligned word across two aligned words");
        
        // Unaligned word at address 0x80000002
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);  // Initial value
        cpu.setRegister(PSX::CPU::S0, 0x80000002);  // Address
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        // Execute LWL first (loads bytes 2-3 into positions 3-2)
        mem.write(0xBFC00000, Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0));
        cpu.executeInstruction();
        
        // Now execute LWR to complete the unaligned word load
        resetPCOnly(cpu);
        address = 0x80000002 + 3;  // End of the word (0x80000005)
        cpu.setRegister(PSX::CPU::S0, address);
        
        mem.write(0xBFC00000, Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0));
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x22116655, 
                    "Combined LWL+LWR should correctly load an unaligned word across two aligned words");
        
        // Unaligned word at address 0x80000003
        cpu.setRegister(PSX::CPU::T0, 0xAABBCCDD);  // Initial value
        cpu.setRegister(PSX::CPU::S0, 0x80000003);  // Address
        
        // Reset PC to ensure instruction execution
        resetPCOnly(cpu);
        
        // Execute LWL first (loads byte 3 into position 3)
        mem.write(0xBFC00000, Encode::I_Type::LWL(PSX::CPU::T0, PSX::CPU::S0, 0));
        cpu.executeInstruction();
        
        // Now execute LWR to complete the unaligned word load
        resetPCOnly(cpu);
        address = 0x80000003 + 3;  // End of the word (0x80000006)
        cpu.setRegister(PSX::CPU::S0, address);
        
        mem.write(0xBFC00000, Encode::I_Type::LWR(PSX::CPU::T0, PSX::CPU::S0, 0));
        cpu.executeInstruction();
        
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x11776655, 
                    "Combined LWL+LWR should correctly load an unaligned word across two aligned words");
    });
    
    // Test for SUBU instruction
    runner.addTest("ALU Operations - SUBU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up test values
        cpu.setRegister(PSX::CPU::T0, 100);
        cpu.setRegister(PSX::CPU::T1, 40);
        cpu.setRegister(PSX::CPU::T2, 0);
        cpu.setRegister(PSX::CPU::T3, 1);
        
        // SUBU - Subtract values
        mem.write(0xBFC00000, Encode::R_Type::SUBU(PSX::CPU::S0, PSX::CPU::T0, PSX::CPU::T1)); // 100 - 40 = 60
        mem.write(0xBFC00004, Encode::R_Type::SUBU(PSX::CPU::S1, PSX::CPU::T1, PSX::CPU::T0)); // 40 - 100 = -60 (as unsigned)
        mem.write(0xBFC00008, Encode::R_Type::SUBU(PSX::CPU::S2, PSX::CPU::T2, PSX::CPU::T3)); // 0 - 1 = -1 (as unsigned)
        
        // Execute the instructions
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // SUBU (100 - 40)
        cpu.executeInstruction();  // SUBU (40 - 100)
        cpu.executeInstruction();  // SUBU (0 - 1)
        
        // Check results
        runner.check(cpu.getRegister(PSX::CPU::S0) == 60, 
                    "SUBU should subtract rt from rs correctly when rs > rt");
                    
        runner.check(cpu.getRegister(PSX::CPU::S1) == 0xFFFFFFC4, // -60 in two's complement 
                    "SUBU should perform subtraction without overflow even if result is negative");
                    
        runner.check(cpu.getRegister(PSX::CPU::S2) == 0xFFFFFFFF, // -1 in two's complement
                    "SUBU should handle underflow correctly");
    });
    
    // Test for LB instruction (Load Byte, sign-extended)
    runner.addTest("Memory Operations - LB/LBU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up test memory with values that will allow testing sign extension
        mem.write(0x80000000, 0x8144FF7F);  // 0x7F: positive max, 0xFF: negative, 0x44: positive, 0x81: negative
        
        cpu.setRegister(PSX::CPU::S0, 0x80000000);
        
        // Test each byte position with LB (sign extension)
        // LB $t0, 0($s0) - Load byte 0 (0x7F - positive)
        mem.write(0xBFC00000, Encode::I_Type::LB(PSX::CPU::T0, PSX::CPU::S0, 0));
        
        // LB $t1, 1($s0) - Load byte 1 (0xFF - negative)
        mem.write(0xBFC00004, Encode::I_Type::LB(PSX::CPU::T1, PSX::CPU::S0, 1));
        
        // LB $t2, 2($s0) - Load byte 2 (0x44 - positive)
        mem.write(0xBFC00008, Encode::I_Type::LB(PSX::CPU::T2, PSX::CPU::S0, 2));
        
        // LB $t3, 3($s0) - Load byte 3 (0x81 - negative)
        mem.write(0xBFC0000C, Encode::I_Type::LB(PSX::CPU::T3, PSX::CPU::S0, 3));
        
        // Test each byte position with LBU (zero extension)
        // LBU $t4, 0($s0) - Load byte 0 unsigned (0x7F)
        mem.write(0xBFC00010, Encode::I_Type::LBU(PSX::CPU::T4, PSX::CPU::S0, 0));
        
        // LBU $t5, 1($s0) - Load byte 1 unsigned (0xFF)
        mem.write(0xBFC00014, Encode::I_Type::LBU(PSX::CPU::T5, PSX::CPU::S0, 1));
        
        // LBU $t6, 2($s0) - Load byte 2 unsigned (0x44)
        mem.write(0xBFC00018, Encode::I_Type::LBU(PSX::CPU::T6, PSX::CPU::S0, 2));
        
        // LBU $t7, 3($s0) - Load byte 3 unsigned (0x81)
        mem.write(0xBFC0001C, Encode::I_Type::LBU(PSX::CPU::T7, PSX::CPU::S0, 3));
        
        // Execute the instructions
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // LB byte 0 (0x7F)
        cpu.executeInstruction();  // LB byte 1 (0xFF)
        cpu.executeInstruction();  // LB byte 2 (0x44)
        cpu.executeInstruction();  // LB byte 3 (0x81)
        cpu.executeInstruction();  // LBU byte 0 (0x7F)
        cpu.executeInstruction();  // LBU byte 1 (0xFF)
        cpu.executeInstruction();  // LBU byte 2 (0x44)
        cpu.executeInstruction();  // LBU byte 3 (0x81)
        
        // Check results for LB (sign extension)
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x0000007F, 
                    "LB should load positive byte (0x7F) correctly without sign extension");
                    
        runner.check(cpu.getRegister(PSX::CPU::T1) == 0xFFFFFFFF, 
                    "LB should sign-extend negative byte (0xFF)");
                    
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0x00000044, 
                    "LB should load positive byte (0x44) correctly without sign extension");
                    
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0xFFFFFF81, 
                    "LB should sign-extend negative byte (0x81)");
        
        // Check results for LBU (zero extension)
        runner.check(cpu.getRegister(PSX::CPU::T4) == 0x0000007F, 
                    "LBU should zero-extend byte (0x7F)");
                    
        runner.check(cpu.getRegister(PSX::CPU::T5) == 0x000000FF, 
                    "LBU should zero-extend byte (0xFF)");
                    
        runner.check(cpu.getRegister(PSX::CPU::T6) == 0x00000044, 
                    "LBU should zero-extend byte (0x44)");
                    
        runner.check(cpu.getRegister(PSX::CPU::T7) == 0x00000081, 
                    "LBU should zero-extend byte (0x81)");
    });
    
    // Test for LH and LHU instructions (Load Halfword with sign/zero extension)
    runner.addTest("Memory Operations - LH/LHU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up test memory with halfwords for testing sign extension
        mem.write(0x80000000, 0x80007FFF);  // 0x7FFF: positive max 16-bit, 0x8000: negative 16-bit
        
        cpu.setRegister(PSX::CPU::S0, 0x80000000);
        
        // Test each halfword position with LH (sign extension)
        // LH $t0, 0($s0) - Load halfword 0 (0x7FFF - positive)
        mem.write(0xBFC00000, Encode::I_Type::LH(PSX::CPU::T0, PSX::CPU::S0, 0));
        
        // LH $t1, 2($s0) - Load halfword 1 (0x8000 - negative)
        mem.write(0xBFC00004, Encode::I_Type::LH(PSX::CPU::T1, PSX::CPU::S0, 2));
        
        // Test with LHU (zero extension)
        // LHU $t2, 0($s0) - Load halfword 0 unsigned (0x7FFF)
        mem.write(0xBFC00008, Encode::I_Type::LHU(PSX::CPU::T2, PSX::CPU::S0, 0));
        
        // LHU $t3, 2($s0) - Load halfword 1 unsigned (0x8000)
        mem.write(0xBFC0000C, Encode::I_Type::LHU(PSX::CPU::T3, PSX::CPU::S0, 2));
        
        // Execute the instructions
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // LH halfword 0 (0x7FFF)
        cpu.executeInstruction();  // LH halfword 1 (0x8000)
        cpu.executeInstruction();  // LHU halfword 0 (0x7FFF)
        cpu.executeInstruction();  // LHU halfword 1 (0x8000)
        
        // Check results for LH (sign extension)
        runner.check(cpu.getRegister(PSX::CPU::T0) == 0x00007FFF, 
                    "LH should load positive halfword (0x7FFF) correctly without sign extension");
                    
        runner.check(cpu.getRegister(PSX::CPU::T1) == 0xFFFF8000, 
                    "LH should sign-extend negative halfword (0x8000)");
        
        // Check results for LHU (zero extension)
        runner.check(cpu.getRegister(PSX::CPU::T2) == 0x00007FFF, 
                    "LHU should zero-extend halfword (0x7FFF)");
                    
        runner.check(cpu.getRegister(PSX::CPU::T3) == 0x00008000, 
                    "LHU should zero-extend halfword (0x8000)");
        
        // Test alignment error
        cpu.setRegister(PSX::CPU::S1, 0x80000001);  // Unaligned address
        
        // LH $t4, 0($s1) - Load halfword from unaligned address
        mem.write(0xBFC00010, Encode::I_Type::LH(PSX::CPU::T4, PSX::CPU::S1, 0));
        
        cpu.executeInstruction();  // LH from unaligned address
        
        // Check exception
        runner.check(cpu.getPC() == 0xBFC00180, 
                    "LH should trigger address error exception for unaligned access");
                    
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x7C) == 
                    (PSX::CPU::EXCEPTION_ADDRESS_ERROR_LOAD << 2), 
                    "Cause register should indicate address error exception");
                    
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_BADVADDR) == 0x80000001, 
                    "BadVAddr should contain the faulting address");
    });
    
    // Test for SB instruction (Store Byte)
    runner.addTest("Memory Operations - SB/SH", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up initial memory value
        mem.write(0x80000000, 0x11223344);
        
        cpu.setRegister(PSX::CPU::S0, 0x80000000);
        cpu.setRegister(PSX::CPU::T0, 0xAA);        // Value to store in byte 0
        cpu.setRegister(PSX::CPU::T1, 0xBB);        // Value to store in byte 1
        cpu.setRegister(PSX::CPU::T2, 0xCC);        // Value to store in byte 2
        cpu.setRegister(PSX::CPU::T3, 0xDD);        // Value to store in byte 3
        
        // Test SB for each byte position
        // SB $t0, 0($s0) - Store byte 0
        mem.write(0xBFC00000, Encode::I_Type::SB(PSX::CPU::T0, PSX::CPU::S0, 0));
        
        // SB $t1, 1($s0) - Store byte 1
        mem.write(0xBFC00004, Encode::I_Type::SB(PSX::CPU::T1, PSX::CPU::S0, 1));
        
        // SB $t2, 2($s0) - Store byte 2
        mem.write(0xBFC00008, Encode::I_Type::SB(PSX::CPU::T2, PSX::CPU::S0, 2));
        
        // SB $t3, 3($s0) - Store byte 3
        mem.write(0xBFC0000C, Encode::I_Type::SB(PSX::CPU::T3, PSX::CPU::S0, 3));
        
        // Execute the instructions
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // SB byte 0
        
        // Check after first byte store
        runner.check(mem.read(0x80000000) == 0x112233AA, 
                    "SB should store byte 0 correctly");
        
        cpu.executeInstruction();  // SB byte 1
        
        // Check after second byte store
        runner.check(mem.read(0x80000000) == 0x1122BBAA, 
                    "SB should store byte 1 correctly");
        
        cpu.executeInstruction();  // SB byte 2
        
        // Check after third byte store
        runner.check(mem.read(0x80000000) == 0x11CCBBAA, 
                    "SB should store byte 2 correctly");
        
        cpu.executeInstruction();  // SB byte 3
        
        // Check after all bytes stored
        runner.check(mem.read(0x80000000) == 0xDDCCBBAA, 
                    "SB should store byte 3 correctly");
        
        // Test SH (Store Halfword)
        // Set up new memory value
        mem.write(0x80000004, 0x55667788);
        
        cpu.setRegister(PSX::CPU::S1, 0x80000004);
        cpu.setRegister(PSX::CPU::T4, 0xABCD);      // Value to store in lower halfword
        cpu.setRegister(PSX::CPU::T5, 0xEF01);      // Value to store in upper halfword
        
        // SH $t4, 0($s1) - Store lower halfword
        mem.write(0xBFC00010, Encode::I_Type::SH(PSX::CPU::T4, PSX::CPU::S1, 0));
        
        // SH $t5, 2($s1) - Store upper halfword
        mem.write(0xBFC00014, Encode::I_Type::SH(PSX::CPU::T5, PSX::CPU::S1, 2));
        
        cpu.executeInstruction();  // SH lower halfword
        
        // Check after lower halfword store
        runner.check(mem.read(0x80000004) == 0x5566ABCD, 
                    "SH should store lower halfword correctly");
        
        cpu.executeInstruction();  // SH upper halfword
        
        // Check after both halfwords stored
        runner.check(mem.read(0x80000004) == 0xEF01ABCD, 
                    "SH should store upper halfword correctly");
        
        // Test alignment error for SH
        cpu.setRegister(PSX::CPU::S2, 0x80000005);  // Unaligned address
        
        // SH $t4, 0($s2) - Store halfword to unaligned address
        mem.write(0xBFC00018, Encode::I_Type::SH(PSX::CPU::T4, PSX::CPU::S2, 0));
        
        cpu.executeInstruction();  // SH to unaligned address
        
        // Check exception
        runner.check(cpu.getPC() == 0xBFC00180, 
                    "SH should trigger address error exception for unaligned access");
                    
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x7C) == 
                    (PSX::CPU::EXCEPTION_ADDRESS_ERROR_STORE << 2), 
                    "Cause register should indicate store address error exception");
                    
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_BADVADDR) == 0x80000005, 
                    "BadVAddr should contain the faulting address");
    });
    
    // Test for XOR and NOR operations
    runner.addTest("ALU Operations - XOR/NOR", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up test values
        cpu.setRegister(PSX::CPU::T0, 0xF0F0F0F0);  // 1111 0000 1111 0000 ...
        cpu.setRegister(PSX::CPU::T1, 0xFF00FF00);  // 1111 1111 0000 0000 ...
        
        // XOR $s0, $t0, $t1 - Should be 0x0FF00FF0
        mem.write(0xBFC00000, Encode::R_Type::XOR(PSX::CPU::S0, PSX::CPU::T0, PSX::CPU::T1));
        
        // NOR $s1, $t0, $t1 - Should be 0x000F000F
        mem.write(0xBFC00004, Encode::R_Type::NOR(PSX::CPU::S1, PSX::CPU::T0, PSX::CPU::T1));
        
        // Execute the instructions
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // XOR
        cpu.executeInstruction();  // NOR
        
        // Check results
        runner.check(cpu.getRegister(PSX::CPU::S0) == 0x0FF00FF0, 
                    "XOR should perform bitwise exclusive OR correctly");
                    
        runner.check(cpu.getRegister(PSX::CPU::S1) == 0x000F000F, 
                    "NOR should perform bitwise NOR correctly (NOT (OR))");
    });
    
    // Test for SLTI and SLTIU operations
    runner.addTest("ALU Operations - SLTI/SLTIU", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up test values
        cpu.setRegister(PSX::CPU::T0, 10);           // Positive small value
        cpu.setRegister(PSX::CPU::T1, 0x80000000);   // Negative value (most negative)
        cpu.setRegister(PSX::CPU::T2, 0x7FFFFFFF);   // Positive value (most positive)
        
        // SLTI tests (signed comparison)
        // SLTI $s0, $t0, 20    - Should be 1 (10 < 20)
        mem.write(0xBFC00000, Encode::I_Type::SLTI(PSX::CPU::S0, PSX::CPU::T0, 20));
        
        // SLTI $s1, $t0, 5     - Should be 0 (10 !< 5)
        mem.write(0xBFC00004, Encode::I_Type::SLTI(PSX::CPU::S1, PSX::CPU::T0, 5));
        
        // SLTI $s2, $t1, 0     - Should be 1 (negative < 0)
        mem.write(0xBFC00008, Encode::I_Type::SLTI(PSX::CPU::S2, PSX::CPU::T1, 0));
        
        // SLTI $s3, $t2, 0     - Should be 0 (positive !< 0)
        mem.write(0xBFC0000C, Encode::I_Type::SLTI(PSX::CPU::S3, PSX::CPU::T2, 0));
        
        // SLTIU tests (unsigned comparison after sign-extension of immediate)
        // SLTIU $s4, $t0, 20   - Should be 1 (10 < 20)
        mem.write(0xBFC00010, Encode::I_Type::SLTIU(PSX::CPU::S4, PSX::CPU::T0, 20));
        
        // SLTIU $s5, $t2, -1   - Should be 1 (0x7FFFFFFF < 0xFFFFFFFF)
        mem.write(0xBFC00014, Encode::I_Type::SLTIU(PSX::CPU::S5, PSX::CPU::T2, -1));
        
        // SLTIU $s6, $t1, 1    - Should be 0 (0x80000000 !< 1 in unsigned comparison)
        mem.write(0xBFC00018, Encode::I_Type::SLTIU(PSX::CPU::S6, PSX::CPU::T1, 1));
        
        // Execute the instructions
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // SLTI (10 < 20)
        cpu.executeInstruction();  // SLTI (10 < 5)
        cpu.executeInstruction();  // SLTI (negative < 0)
        cpu.executeInstruction();  // SLTI (positive < 0)
        cpu.executeInstruction();  // SLTIU (10 < 20)
        cpu.executeInstruction();  // SLTIU (0x7FFFFFFF < 0xFFFFFFFF)
        cpu.executeInstruction();  // SLTIU (0x80000000 < 1)
        
        // Check results
        runner.check(cpu.getRegister(PSX::CPU::S0) == 1, 
                    "SLTI should set destination register to 1 when rs < immediate (10 < 20)");
                    
        runner.check(cpu.getRegister(PSX::CPU::S1) == 0, 
                    "SLTI should set destination register to 0 when rs >= immediate (10 >= 5)");
                    
        runner.check(cpu.getRegister(PSX::CPU::S2) == 1, 
                    "SLTI should correctly compare negative values (0x80000000 < 0)");
                    
        runner.check(cpu.getRegister(PSX::CPU::S3) == 0, 
                    "SLTI should correctly compare positive values (0x7FFFFFFF >= 0)");
                    
        runner.check(cpu.getRegister(PSX::CPU::S4) == 1, 
                    "SLTIU should set destination register to 1 when rs < immediate (10 < 20)");
                    
        runner.check(cpu.getRegister(PSX::CPU::S5) == 1, 
                    "SLTIU should compare correctly after sign-extension (0x7FFFFFFF < 0xFFFFFFFF)");
                    
        runner.check(cpu.getRegister(PSX::CPU::S6) == 0, 
                    "SLTIU should compare as unsigned (0x80000000 !< 1)");
    });
    
    // Test for SYSCALL instruction
    runner.addTest("System Instructions - SYSCALL", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Write SYSCALL instruction to memory
        mem.write(0xBFC00000, Encode::R_Type::SYSCALL());
        
        // Execute the instruction
        cpu.setPC(0xBFC00000);
        
        // Print initial state
        std::cout << "Before SYSCALL - PC: 0x" << std::hex << cpu.getPC() << std::endl;
        
        cpu.executeInstruction();  // SYSCALL
        
        // Print state after exception
        std::cout << "After SYSCALL - PC: 0x" << std::hex << cpu.getPC() << std::endl;
        std::cout << "EPC: 0x" << std::hex << cpu.getCP0Register(PSX::CPU::CP0_EPC) << std::endl;
        std::cout << "Cause: 0x" << std::hex << cpu.getCP0Register(PSX::CPU::CP0_CAUSE) << std::endl;
        
        // Check if exception was triggered properly
        runner.check(cpu.getPC() == 0xBFC00180,  // Default exception vector
                    "SYSCALL should trigger exception and jump to exception vector");
                    
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x7C) == 
                    (PSX::CPU::EXCEPTION_SYSCALL << 2), 
                    "Cause register should indicate syscall exception");
                    
        // In our implementation, the PC is already advanced to the next instruction
        // when the exception is triggered, so EPC should be the current PC
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_EPC) == 0xBFC00000, 
                    "EPC should contain the address of the SYSCALL instruction");
                    
        // Check that the BD (Branch Delay) bit is not set
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x80000000) == 0,
                    "BD bit should not be set for a normal exception");
    });
    
    // Test for BLEZ and BGTZ instructions
    runner.addTest("Branch Instructions - BLEZ/BGTZ", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Test BLEZ with negative value
        cpu.setRegister(PSX::CPU::T0, -5);  // Negative value
        
        // blez $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BLEZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute blez
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00044, 
                    "BLEZ should branch when value is negative");
        
        // Test BLEZ with zero value
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, 0);  // Zero value
        
        // blez $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BLEZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute blez
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00044, 
                    "BLEZ should branch when value is zero");
        
        // Test BLEZ with positive value (should not branch)
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, 5);  // Positive value
        
        // blez $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BLEZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute blez
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00008, 
                    "BLEZ should not branch when value is positive");
        
        // Test BGTZ with positive value
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, 5);  // Positive value
        
        // bgtz $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BGTZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bgtz
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00044, 
                    "BGTZ should branch when value is positive");
        
        // Test BGTZ with zero value (should not branch)
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, 0);  // Zero value
        
        // bgtz $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BGTZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bgtz
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00008, 
                    "BGTZ should not branch when value is zero");
        
        // Test BGTZ with negative value (should not branch)
        cpu.reset();
        setupCPUWithMemory(cpu, mem);
        cpu.setRegister(PSX::CPU::T0, -5);  // Negative value
        
        // bgtz $t0, 0x10 (16 instructions forward)
        mem.write(0xBFC00000, Encode::I_Type::BGTZ(PSX::CPU::T0, 0x10));
        // nop
        mem.write(0xBFC00004, 0x00000000);
        
        cpu.executeInstruction();  // Execute bgtz
        cpu.executeInstruction();  // Execute delay slot
        
        runner.check(cpu.getPC() == 0xBFC00008, 
                    "BGTZ should not branch when value is negative");
    });

    // Test for BREAK instruction
    runner.addTest("System Instructions - BREAK", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Write BREAK instruction to memory
        mem.write(0xBFC00000, Encode::R_Type::BREAK());
        
        // Execute the instruction
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // BREAK
        
        // Check if exception was triggered properly
        runner.check(cpu.getPC() == 0xBFC00180,  // Default exception vector
                    "BREAK should trigger exception and jump to exception vector");
                    
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x7C) == 
                    (PSX::CPU::EXCEPTION_BREAKPOINT << 2), 
                    "Cause register should indicate breakpoint exception");
                    
        runner.check(cpu.getCP0Register(PSX::CPU::CP0_EPC) == 0xBFC00000, 
                    "EPC should contain the address of the BREAK instruction");
                    
        // Check that the BD (Branch Delay) bit is not set
        runner.check((cpu.getCP0Register(PSX::CPU::CP0_CAUSE) & 0x80000000) == 0,
                    "BD bit should not be set for a normal exception");
    });

    // Test for RFE instruction
    runner.addTest("System Instructions - RFE", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Set up Status register with test values
        // IEc=0, KUc=0, IEp=1, KUp=1, IEo=0, KUo=0
        cpu.setCP0Register(PSX::CPU::CP0_SR, 0x0C); // 0b1100
        
        // Write RFE instruction to memory
        mem.write(0xBFC00000, Encode::R_Type::RFE());
        
        // Execute the instruction
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // RFE
        
        // Check if Status register was updated correctly
        // After RFE: IEc=1, KUc=1, IEp=0, KUp=0, IEo=0, KUo=0
        uint32_t status = cpu.getCP0Register(PSX::CPU::CP0_SR);
        
        // The proper expected result is 0x03
        runner.check(status == 0x03, 
                    "RFE should shift KUp/IEp bits to KUc/IEc position and clear other bits");
                    
        // Check PC advanced properly (not an exception)
        runner.check(cpu.getPC() == 0xBFC00004, 
                    "PC should advance to next instruction after RFE");
                    
        // Test another bit pattern (ensure only the relevant bits shift)
        // Set SR with additional bits set (0xFF0C = 0b1111 1111 0000 1100)
        cpu.setCP0Register(PSX::CPU::CP0_SR, 0xFF0C); 
        
        // Reset PC and write RFE instruction again
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // RFE
        
        status = cpu.getCP0Register(PSX::CPU::CP0_SR);
        
        // The proper expected result is 0xFF03
        runner.check(status == 0xFF03, 
                    "RFE should only shift the mode bits and preserve other bits");
                    
        // Test for complete bit shifting behavior including old bits to previous bits
        // Set SR with values in all three modes:
        // IEc=0, KUc=0 (bits 1-0 = 00)
        // IEp=1, KUp=1 (bits 3-2 = 11) 
        // IEo=1, KUo=0 (bits 5-4 = 10)
        cpu.setCP0Register(PSX::CPU::CP0_SR, 0x2C); // 0b0010 1100
        std::cout << "Before 3rd RFE - SR: 0x" << std::hex << cpu.getCP0Register(PSX::CPU::CP0_SR) << std::endl;
        
        // Reset PC and write RFE instruction again
        cpu.setPC(0xBFC00000);
        cpu.executeInstruction();  // RFE
        
        status = cpu.getCP0Register(PSX::CPU::CP0_SR);
        std::cout << "After 3rd RFE - SR: 0x" << std::hex << status << std::endl;
        
        // After RFE:
        // - Bits 1-0 should contain 0b11 (from bits 3-2)
        // - Bits 3-2 should contain 0b10 (from bits 5-4)
        // - Bits 5-4 should remain 0b10
        // Expected result is 0x2B (0b0010 1011)
        runner.check(status == 0x2B, 
                    "RFE should shift old mode bits to previous mode bits");
    });

    // RFE instruction test
    runner.addTest("RFE Instruction", [&runner]() {
        PSX::CPU cpu;
        TestMemory mem;
        setupCPUWithMemory(cpu, mem);
        
        // Setup Status register with 0x2C (0b00101100)
        // bits 5-4 (old mode): 0b01 (1)
        // bits 3-2 (previous mode): 0b01 (1)
        // bits 1-0 (current mode): 0b00 (0)
        cpu.setCP0Register(PSX::CPU::CP0_SR, 0x2C);
        
        // RFE instruction
        uint32_t rfeInst = Encode::R_Type::RFE();
        mem.write(0xBFC00000, rfeInst);
        
        // Execute the instruction
        cpu.executeInstruction();
        
        // Expected result after RFE:
        // bits 5-4 (old mode): 0b01 (1) - unchanged
        // bits 3-2 (previous mode): 0b01 (1) - gets value from old mode
        // bits 1-0 (current mode): 0b01 (1) - gets value from previous mode
        // Result should be 0b00101011 (0x2B)
        uint32_t result = cpu.getCP0Register(PSX::CPU::CP0_SR);
        runner.check(result == 0x2B, "RFE should shift mode bits correctly");
    });

    runner.runAll();
}

} // namespace PSXTest

// Main function to run the tests
int main() {
    std::cout << "Running PlayStation CPU Emulator Tests\n";
    PSXTest::runTests();
    return 0;
}