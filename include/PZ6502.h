#pragma once

#include <iostream>
#include <map>  // for disasember
#include <vector> // store INSTRUCTIONS

//! Forward declaration of generic communication
//! bus class to prevent circular inclusion
class Bus;

// reference: https://www.mdawson.net/vic20chrome/cpu/mos_6500_mpu_preliminary_may_1976.pdf
class PZ6502{
    public:
        PZ6502();
        ~PZ6502();
        void ConnectBus(Bus *n){ m_bus = n; }

        uint8_t fetched = 0x00;   // store the fetched data
        uint16_t addrAbs = 0x0000;
        uint16_t addrRel = 0x0000;
        uint8_t opcode = 0x00;
        uint8_t cycles = 0;

    public:
        void clock();
        void reset();
        void irq();         // interrupt request
        void nmi();         // non-maskable interrupt
        uint8_t fetch();    // used to fetch the data
        bool complete();  // when cycles equals 0
        std::map<uint16_t, std::string> disassemble(uint16_t, uint16_t);
        
    public:
        enum FLAGS6502{
            C = (1 << 0),   // Carry bit
            Z = (1 << 1),   // Zero
            I = (1 << 2),   // Disable Interrupts
            D = (1 << 3),   // Decimal Mode
            B = (1 << 4),   // Break
            U = (1 << 5),   // Unused
            V = (1 << 6),   // Overflow
            N = (1 << 7),   // Negative
        };

        uint8_t accum = 0x00;     // Accumulator register
        uint8_t x_reg = 0x00;      // X register
        uint8_t y_reg = 0x00;      // Y register
        uint8_t stkp = 0x00;      // Stack Pointer (points to location on bus)
        uint16_t pc = 0x0000;     // Program counter
        uint8_t status = 0x00;    // status register


    private:
        Bus     *m_bus = nullptr;
        uint8_t read(uint16_t addr);
        void    write(uint16_t addr, uint8_t dat);

        uint8_t GetFlag(FLAGS6502 f);
        void    SetFlag(FLAGS6502 f, bool v);


        // addressing mode
        uint8_t IMP();      // implied addressing
        uint8_t IMM();      // immediate ""
        uint8_t ZP0();      // zero page ""
        uint8_t ZPX();      // indexed Zero page (X)
        uint8_t ZPY();      // indexed Zero page (Y)
        uint8_t REL();
        uint8_t ABS();      // absolute ""
        uint8_t ABX();      // indexed absolute "" (X)
        uint8_t ABY();      // indexed absolute "" (Y)
        uint8_t IND();      // absolute indirect
        uint8_t IZX();      // indexed indirect ""
        uint8_t IZY();      // indirect indexed ""

        // Opcodes
        uint8_t ADC();	uint8_t AND();	uint8_t ASL();	uint8_t BCC();
        uint8_t BCS();	uint8_t BEQ();	uint8_t BIT();	uint8_t BMI();
        uint8_t BNE();	uint8_t BPL();	uint8_t BRK();	uint8_t BVC();
        uint8_t BVS();	uint8_t CLC();	uint8_t CLD();	uint8_t CLI();
        uint8_t CLV();	uint8_t CMP();	uint8_t CPX();	uint8_t CPY();
        uint8_t DEC();	uint8_t DEX();	uint8_t DEY();	uint8_t EOR();
        uint8_t INC();	uint8_t INX();	uint8_t INY();	uint8_t JMP();
        uint8_t JSR();	uint8_t LDA();	uint8_t LDX();	uint8_t LDY();
        uint8_t LSR();	uint8_t NOP();	uint8_t ORA();	uint8_t PHA();
        uint8_t PHP();	uint8_t PLA();	uint8_t PLP();	uint8_t ROL();
        uint8_t ROR();	uint8_t RTI();	uint8_t RTS();	uint8_t SBC();
        uint8_t SEC();	uint8_t SED();	uint8_t SEI();	uint8_t STA();
        uint8_t STX();	uint8_t STY();	uint8_t TAX();	uint8_t TAY();
        uint8_t TSX();	uint8_t TXA();	uint8_t TXS();	uint8_t TYA();

        // invalid opcodes for this cpu
        uint8_t XXX();

        struct INSTRUCTION{
            std::string name; //mnemonic
            uint8_t     (PZ6502::*operate)() = nullptr;  // function pointer to opcode
            uint8_t     (PZ6502::*addrmode)() = nullptr;    // function pointer to address mode
            uint8_t     cycle = 0;
        };

        std::vector<INSTRUCTION> m_lookup;
};