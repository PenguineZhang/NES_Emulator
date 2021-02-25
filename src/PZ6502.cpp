#include "Bus.h"
#include "PZ6502.h"

PZ6502::PZ6502(){
    using pz = PZ6502;

    m_lookup = {
        { "BRK", &pz::BRK, &pz::IMM, 7 },{ "ORA", &pz::ORA, &pz::IZX, 6 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 3 },{ "ORA", &pz::ORA, &pz::ZP0, 3 },{ "ASL", &pz::ASL, &pz::ZP0, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "PHP", &pz::PHP, &pz::IMP, 3 },{ "ORA", &pz::ORA, &pz::IMM, 2 },{ "ASL", &pz::ASL, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "ORA", &pz::ORA, &pz::ABS, 4 },{ "ASL", &pz::ASL, &pz::ABS, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },
		{ "BPL", &pz::BPL, &pz::REL, 2 },{ "ORA", &pz::ORA, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "ORA", &pz::ORA, &pz::ZPX, 4 },{ "ASL", &pz::ASL, &pz::ZPX, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "CLC", &pz::CLC, &pz::IMP, 2 },{ "ORA", &pz::ORA, &pz::ABY, 4 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 7 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "ORA", &pz::ORA, &pz::ABX, 4 },{ "ASL", &pz::ASL, &pz::ABX, 7 },{ "???", &pz::XXX, &pz::IMP, 7 },
		{ "JSR", &pz::JSR, &pz::ABS, 6 },{ "AND", &pz::AND, &pz::IZX, 6 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "BIT", &pz::BIT, &pz::ZP0, 3 },{ "AND", &pz::AND, &pz::ZP0, 3 },{ "ROL", &pz::ROL, &pz::ZP0, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "PLP", &pz::PLP, &pz::IMP, 4 },{ "AND", &pz::AND, &pz::IMM, 2 },{ "ROL", &pz::ROL, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "BIT", &pz::BIT, &pz::ABS, 4 },{ "AND", &pz::AND, &pz::ABS, 4 },{ "ROL", &pz::ROL, &pz::ABS, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },
		{ "BMI", &pz::BMI, &pz::REL, 2 },{ "AND", &pz::AND, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "AND", &pz::AND, &pz::ZPX, 4 },{ "ROL", &pz::ROL, &pz::ZPX, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "SEC", &pz::SEC, &pz::IMP, 2 },{ "AND", &pz::AND, &pz::ABY, 4 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 7 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "AND", &pz::AND, &pz::ABX, 4 },{ "ROL", &pz::ROL, &pz::ABX, 7 },{ "???", &pz::XXX, &pz::IMP, 7 },
		{ "RTI", &pz::RTI, &pz::IMP, 6 },{ "EOR", &pz::EOR, &pz::IZX, 6 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 3 },{ "EOR", &pz::EOR, &pz::ZP0, 3 },{ "LSR", &pz::LSR, &pz::ZP0, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "PHA", &pz::PHA, &pz::IMP, 3 },{ "EOR", &pz::EOR, &pz::IMM, 2 },{ "LSR", &pz::LSR, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "JMP", &pz::JMP, &pz::ABS, 3 },{ "EOR", &pz::EOR, &pz::ABS, 4 },{ "LSR", &pz::LSR, &pz::ABS, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },
		{ "BVC", &pz::BVC, &pz::REL, 2 },{ "EOR", &pz::EOR, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "EOR", &pz::EOR, &pz::ZPX, 4 },{ "LSR", &pz::LSR, &pz::ZPX, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "CLI", &pz::CLI, &pz::IMP, 2 },{ "EOR", &pz::EOR, &pz::ABY, 4 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 7 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "EOR", &pz::EOR, &pz::ABX, 4 },{ "LSR", &pz::LSR, &pz::ABX, 7 },{ "???", &pz::XXX, &pz::IMP, 7 },
		{ "RTS", &pz::RTS, &pz::IMP, 6 },{ "ADC", &pz::ADC, &pz::IZX, 6 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 3 },{ "ADC", &pz::ADC, &pz::ZP0, 3 },{ "ROR", &pz::ROR, &pz::ZP0, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "PLA", &pz::PLA, &pz::IMP, 4 },{ "ADC", &pz::ADC, &pz::IMM, 2 },{ "ROR", &pz::ROR, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "JMP", &pz::JMP, &pz::IND, 5 },{ "ADC", &pz::ADC, &pz::ABS, 4 },{ "ROR", &pz::ROR, &pz::ABS, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },
		{ "BVS", &pz::BVS, &pz::REL, 2 },{ "ADC", &pz::ADC, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "ADC", &pz::ADC, &pz::ZPX, 4 },{ "ROR", &pz::ROR, &pz::ZPX, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "SEI", &pz::SEI, &pz::IMP, 2 },{ "ADC", &pz::ADC, &pz::ABY, 4 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 7 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "ADC", &pz::ADC, &pz::ABX, 4 },{ "ROR", &pz::ROR, &pz::ABX, 7 },{ "???", &pz::XXX, &pz::IMP, 7 },
		{ "???", &pz::NOP, &pz::IMP, 2 },{ "STA", &pz::STA, &pz::IZX, 6 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "STY", &pz::STY, &pz::ZP0, 3 },{ "STA", &pz::STA, &pz::ZP0, 3 },{ "STX", &pz::STX, &pz::ZP0, 3 },{ "???", &pz::XXX, &pz::IMP, 3 },{ "DEY", &pz::DEY, &pz::IMP, 2 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "TXA", &pz::TXA, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "STY", &pz::STY, &pz::ABS, 4 },{ "STA", &pz::STA, &pz::ABS, 4 },{ "STX", &pz::STX, &pz::ABS, 4 },{ "???", &pz::XXX, &pz::IMP, 4 },
		{ "BCC", &pz::BCC, &pz::REL, 2 },{ "STA", &pz::STA, &pz::IZY, 6 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "STY", &pz::STY, &pz::ZPX, 4 },{ "STA", &pz::STA, &pz::ZPX, 4 },{ "STX", &pz::STX, &pz::ZPY, 4 },{ "???", &pz::XXX, &pz::IMP, 4 },{ "TYA", &pz::TYA, &pz::IMP, 2 },{ "STA", &pz::STA, &pz::ABY, 5 },{ "TXS", &pz::TXS, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "???", &pz::NOP, &pz::IMP, 5 },{ "STA", &pz::STA, &pz::ABX, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },
		{ "LDY", &pz::LDY, &pz::IMM, 2 },{ "LDA", &pz::LDA, &pz::IZX, 6 },{ "LDX", &pz::LDX, &pz::IMM, 2 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "LDY", &pz::LDY, &pz::ZP0, 3 },{ "LDA", &pz::LDA, &pz::ZP0, 3 },{ "LDX", &pz::LDX, &pz::ZP0, 3 },{ "???", &pz::XXX, &pz::IMP, 3 },{ "TAY", &pz::TAY, &pz::IMP, 2 },{ "LDA", &pz::LDA, &pz::IMM, 2 },{ "TAX", &pz::TAX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "LDY", &pz::LDY, &pz::ABS, 4 },{ "LDA", &pz::LDA, &pz::ABS, 4 },{ "LDX", &pz::LDX, &pz::ABS, 4 },{ "???", &pz::XXX, &pz::IMP, 4 },
		{ "BCS", &pz::BCS, &pz::REL, 2 },{ "LDA", &pz::LDA, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "LDY", &pz::LDY, &pz::ZPX, 4 },{ "LDA", &pz::LDA, &pz::ZPX, 4 },{ "LDX", &pz::LDX, &pz::ZPY, 4 },{ "???", &pz::XXX, &pz::IMP, 4 },{ "CLV", &pz::CLV, &pz::IMP, 2 },{ "LDA", &pz::LDA, &pz::ABY, 4 },{ "TSX", &pz::TSX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 4 },{ "LDY", &pz::LDY, &pz::ABX, 4 },{ "LDA", &pz::LDA, &pz::ABX, 4 },{ "LDX", &pz::LDX, &pz::ABY, 4 },{ "???", &pz::XXX, &pz::IMP, 4 },
		{ "CPY", &pz::CPY, &pz::IMM, 2 },{ "CMP", &pz::CMP, &pz::IZX, 6 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "CPY", &pz::CPY, &pz::ZP0, 3 },{ "CMP", &pz::CMP, &pz::ZP0, 3 },{ "DEC", &pz::DEC, &pz::ZP0, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "INY", &pz::INY, &pz::IMP, 2 },{ "CMP", &pz::CMP, &pz::IMM, 2 },{ "DEX", &pz::DEX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "CPY", &pz::CPY, &pz::ABS, 4 },{ "CMP", &pz::CMP, &pz::ABS, 4 },{ "DEC", &pz::DEC, &pz::ABS, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },
		{ "BNE", &pz::BNE, &pz::REL, 2 },{ "CMP", &pz::CMP, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "CMP", &pz::CMP, &pz::ZPX, 4 },{ "DEC", &pz::DEC, &pz::ZPX, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "CLD", &pz::CLD, &pz::IMP, 2 },{ "CMP", &pz::CMP, &pz::ABY, 4 },{ "NOP", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 7 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "CMP", &pz::CMP, &pz::ABX, 4 },{ "DEC", &pz::DEC, &pz::ABX, 7 },{ "???", &pz::XXX, &pz::IMP, 7 },
		{ "CPX", &pz::CPX, &pz::IMM, 2 },{ "SBC", &pz::SBC, &pz::IZX, 6 },{ "???", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "CPX", &pz::CPX, &pz::ZP0, 3 },{ "SBC", &pz::SBC, &pz::ZP0, 3 },{ "INC", &pz::INC, &pz::ZP0, 5 },{ "???", &pz::XXX, &pz::IMP, 5 },{ "INX", &pz::INX, &pz::IMP, 2 },{ "SBC", &pz::SBC, &pz::IMM, 2 },{ "NOP", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::SBC, &pz::IMP, 2 },{ "CPX", &pz::CPX, &pz::ABS, 4 },{ "SBC", &pz::SBC, &pz::ABS, 4 },{ "INC", &pz::INC, &pz::ABS, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },
		{ "BEQ", &pz::BEQ, &pz::REL, 2 },{ "SBC", &pz::SBC, &pz::IZY, 5 },{ "???", &pz::XXX, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 8 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "SBC", &pz::SBC, &pz::ZPX, 4 },{ "INC", &pz::INC, &pz::ZPX, 6 },{ "???", &pz::XXX, &pz::IMP, 6 },{ "SED", &pz::SED, &pz::IMP, 2 },{ "SBC", &pz::SBC, &pz::ABY, 4 },{ "NOP", &pz::NOP, &pz::IMP, 2 },{ "???", &pz::XXX, &pz::IMP, 7 },{ "???", &pz::NOP, &pz::IMP, 4 },{ "SBC", &pz::SBC, &pz::ABX, 4 },{ "INC", &pz::INC, &pz::ABX, 7 },{ "???", &pz::XXX, &pz::IMP, 7 },
    };
}

PZ6502::~PZ6502(){

}

uint8_t PZ6502::read(uint16_t addr){
    return m_bus->read(addr, false);
}


void PZ6502::write(uint16_t addr, uint8_t dat){
    m_bus->write(addr, dat);
}


uint8_t PZ6502::GetFlag(FLAGS6502 f){

}

/**
 * Set or clear bit in register
 *
 * @param[in] FLAGS6502
 *  f: flag bit wanted to be set/cleared
 * @param[in] bool
 *  v: set/clear indicator. 1 = set, 0 = clear
 */
void PZ6502::SetFlag(FLAGS6502 f, bool v){
    if(v){
        m_status |= f;
    }else{
        m_status &= ~f;
    }
}

void PZ6502::clock(){
    if(m_cycles == 0){
        m_opcode = read(m_pc);
        m_pc++;

        // get starting number of cycles
        m_cycles = m_lookup[m_opcode].cycle;

        // fetch intermediate data
        uint8_t additional_cycle1 = (this->*m_lookup[m_opcode].addrmode)();

        // perform operation
        uint8_t additional_cycle2 = (this->*m_lookup[m_opcode].operate)();

        m_cycles += (additional_cycle1 & additional_cycle2);
    }

    m_cycles--;
}

// addressing mode
uint8_t PZ6502::IMP(){
    m_fetched = m_accum;
    return 0;
}

uint8_t PZ6502::IMM(){
    m_addrAbs = m_pc++;
    return 0;
}

uint8_t PZ6502::ZP0(){
    m_addrAbs = read(m_pc);
    m_pc++;
    m_addrAbs &= 0x00ff;
    return 0;
}

uint8_t PZ6502::ZPX(){
    m_addrAbs = read(m_pc) + m_xReg;
    m_pc++;
    m_addrAbs &= 0x00ff;
    return 0;
}

uint8_t PZ6502::ZPY(){
    m_addrAbs = read(m_pc) + m_yReg;
    m_pc++;
    m_addrAbs &= 0x00ff;
    return 0;
}

uint8_t PZ6502::REL(){
    m_addrRel = read(m_pc);
    m_pc++;

    if(m_addrRel & 0x80){
        m_addrRel |= 0xff00;
    }

    return 0;
}

uint8_t PZ6502::ABS(){
    uint16_t lo = read(m_pc);
    m_pc++;
    uint16_t hi = read(m_pc);
    m_pc++;

    m_addrAbs = (hi << 8) | lo;
    return 0;
}

uint8_t PZ6502::ABX(){
    uint16_t lo = read(m_pc);
    m_pc++;
    uint16_t hi = read(m_pc);
    m_pc++;

    m_addrAbs = (hi << 8) | lo;
    m_addrAbs += m_xReg;

    if((m_addrAbs & 0xff00) != (hi << 8)){
        return 1;
    }else{
        return 0;
    }
}

uint8_t PZ6502::ABY(){
    uint16_t lo = read(m_pc);
    m_pc++;
    uint16_t hi = read(m_pc);
    m_pc++;

    m_addrAbs = (hi << 8) | lo;
    m_addrAbs += m_yReg;

    if((m_addrAbs & 0xff00) != (hi << 8)){
        return 1;
    }else{
        return 0;
    }
}

uint8_t PZ6502::IND(){
    uint16_t ptr_lo = read(m_pc);
    m_pc++;
    uint16_t ptr_hi = read(m_pc);
    m_pc++;

    uint16_t ptr = (ptr_hi << 8) | ptr_lo;

    if(ptr_lo == 0x00ff){
        m_addrAbs = read((ptr & 0xff00) << 8) | read(ptr + 0);
    }else{
        m_addrAbs = (read(ptr + 1) << 8) | read(ptr + 0);
    }

    return 0;
}

uint8_t PZ6502::IZX(){
    uint16_t t = read(m_pc);
    m_pc++;

    uint16_t lo = read((uint16_t)(t + (uint16_t)m_xReg) & 0x00ff);
    uint16_t hi = read((uint16_t)(t + (uint16_t)m_xReg + 1) & 0xff00);

    m_addrAbs = (hi << 8) | lo;

    return 0;
}

uint8_t PZ6502::IZY(){
    uint16_t t = read(m_pc);
    m_pc++;

    uint16_t lo = read(t & 0x00ff);
    uint16_t hi = read((t + 1) & 0x00ff);

    m_addrAbs = (hi << 8) | lo;
    m_addrAbs += m_yReg;

    if((m_addrAbs & 0xff00) != (hi << 8)){
        return 1;
    }else{
        return 0;
    }
}

uint8_t PZ6502::fetch(){
    if(!(m_lookup[m_opcode].addrmode == &PZ6502::IMP)){
        m_fetched = read(m_addrAbs);
    }

    return m_fetched;
}


uint8_t PZ6502::BCS(){
    if(GetFlag(FLAGS6502::C) == 1){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::BCC(){
    if(GetFlag(FLAGS6502::C) == 0){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::BEQ(){
    if(GetFlag(FLAGS6502::Z) == 1){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::BNE(){
    if(GetFlag(FLAGS6502::Z) == 0){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::BMI(){
    if(GetFlag(FLAGS6502::N) == 1){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}



uint8_t PZ6502::BPL(){
    if(GetFlag(FLAGS6502::N) == 0){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::BVC(){
    if(GetFlag(FLAGS6502::V) == 1){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::BVS(){
    if(GetFlag(FLAGS6502::V) == 0){
        m_cycles++;
        m_addrAbs = m_pc + m_addrRel;

        if((m_addrAbs & 0xff00) != (m_pc & 0xff00)){
            m_cycles++;
        }

        m_pc = m_addrAbs;
    }

    return 0;
}


uint8_t PZ6502::CLC(){
    SetFlag(FLAGS6502::C, false);
    return 0;
}


uint8_t PZ6502::CLD(){
    SetFlag(FLAGS6502::D, false);
    return 0;
}


uint8_t PZ6502::AND(){
    fetch();
    m_accum = m_accum & m_fetched;

    SetFlag(FLAGS6502::Z, m_accum == 0x00);
    SetFlag(FLAGS6502::N, m_accum & 0x80);

    return 1;
}


/**
 * Truth table:
 *  A - accumulator
 *  M - fetched value
 *  R - result
 *  V - overflow bit
 *
 *  0 - positive
 *  1 - negative
 *
 *  A  M  R  N   A^R  ~(A^M)
 *  0  0  0  0    0      1
 *  0  0  1  1    1      1
 *  0  1  0  0    0      0
 *  0  1  1  0    1      0
 *  1  0  0  0    1      0         V = (A^R) & ~(A^M)
 *  1  0  1  0    0      0
 *  1  1  0  1    1      1
 *  1  1  1  0    0      1
 */
uint8_t PZ6502::ADC(){
    fetch();
    uint16_t temp = (uint16_t)m_accum + (uint16_t)m_fetched + (uint16_t)GetFlag(C);
    SetFlag(C, temp > 255);
    SetFlag(Z, (temp & 0x00ff) == 0);
    SetFlag(N, temp & 0x80);
    SetFlag(V, ( ((uint16_t)m_accum ^ temp) & ~((uint16_t)m_accum ^ (uint16_t)m_fetched) ) );

    m_accum = temp & 0x00ff;
    return 1;
}


/**
 * reference: https://www.masswerk.at/6502/6502_instruction_set.html#SBC
 *
 * Using 2's compliment
 * A = A - M - (1-C)
 *         _
 * A = A + M + 1 - 1 + C
 *         _
 * A = A + M + C
 */
uint8_t PZ6502::SBC(){
    fetch();

    // invert the data
    uint16_t inverted_M = ((uint16_t)m_fetched) ^ 0x00ff;

    uint16_t temp = (uint16_t)m_accum + inverted_M + (uint16_t)GetFlag(C);
    SetFlag(C, temp & 0xff00);
    SetFlag(Z, (temp & 0x00ff) == 0);
    SetFlag(N, temp & 0x80);
    SetFlag(V, ( ((uint16_t)m_accum ^ temp) & ~((uint16_t)m_accum ^ inverted_M) ) );

    m_accum = temp & 0x00ff;
    return 1;
}

// push accumulator on stack
uint8_t PZ6502::PHA(){
    write(0x0100 + m_stkp, m_accum);
    m_stkp--;
    return 0;
}