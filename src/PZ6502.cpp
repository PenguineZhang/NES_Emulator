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
    return (status & f) == 0 ? 0 : 1;
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
        status |= f;
    }else{
        status &= ~f;
    }
}

void PZ6502::clock(){
    if(cycles == 0){
        opcode = read(pc);
        pc++;

        // get starting number of cycles
        cycles = m_lookup[opcode].cycle;

        // perform addrmode to get the memory address for operand
        uint8_t additional_cycle1 = (this->*m_lookup[opcode].addrmode)();

        // perform operation
        uint8_t additional_cycle2 = (this->*m_lookup[opcode].operate)();

        cycles += (additional_cycle1 & additional_cycle2);
    }

    cycles--;
}

// addressing mode

/**
 * Implicit
 *
 * For many 6502 instructions the source and destination
 * of the information to be manipulated is implied directly
 * by the function of the instruction itself and no further
 *  operand needs to be specified. Operations like 'Clear Carry Flag'
 * (CLC) and 'Return from Subroutine' (RTS) are implicit.
 */
uint8_t PZ6502::IMP(){
    fetched = accum;
    return 0;
}


/**
 * Immediate
 *
 * Immediate addressing allows the programmer to directly specify
 * an 8 bit constant within the instruction. It is indicated by
 * a '#' symbol followed by an numeric expression
 */
uint8_t PZ6502::IMM(){
    addrAbs = pc++;
    return 0;
}


/**
 * Zero Page
 *
 * This limits it to addressing only the first 256 bytes of
 * memory (e.g. $0000 to $00FF) where the most significant
 * byte of the address is always zero. In zero page mode only
 * the least significant byte of the address is held in the
 * instruction making it shorter by one byte (important for
 * space saving) and one less memory fetch during execution (important for speed).
 *
 * An assembler will automatically select zero page addressing
 * mode if the operand evaluates to a zero page address and the
 * instruction supports the mode (not all do).
 */
uint8_t PZ6502::ZP0(){
    addrAbs = read(pc);
    pc++;
    addrAbs &= 0x00ff;
    return 0;
}

/**
 * Zero Page,X
 *
 * The address to be accessed by an instruction using
 * indexed zero page addressing is calculated by taking
 * the 8 bit zero page address from the instruction and
 * adding the current value of the X register to it.
 * For example if the X register contains $0F and
 * the instruction LDA $80,X is executed then the accumulator
 * will be loaded from $008F (e.g. $80 + $0F => $8F).
 *
 * The address calculation wraps around if the sum of the base
 * address and the register exceed $FF. If we repeat the last
 * example but with $FF in the X register then the accumulator
 * will be loaded from $007F (e.g. $80 + $FF => $7F) and not $017F.
 *
 */
uint8_t PZ6502::ZPX(){
    addrAbs = read(pc) + x_reg;
    pc++;
    addrAbs &= 0x00ff;
    return 0;
}

/**
 * Zero Page,Y
 *
 * The address to be accessed by an instruction using
 * indexed zero page addressing is calculated by taking
 * the 8 bit zero page address from the instruction and
 * adding the current value of the Y register to it.
 * This mode can only be used with the LDX and STX instructions.
 */
uint8_t PZ6502::ZPY(){
    addrAbs = read(pc) + y_reg;
    pc++;
    addrAbs &= 0x00ff;
    return 0;
}


/**
 * Relative
 *
 * Relative addressing mode is used by branch
 * instructions (e.g. BEQ, BNE, etc.) which contain a
 * signed 8 bit relative offset (e.g. -128 to +127) which
 * is added to program counter if the condition is true.
 * As the program counter itself is incremented during
 * instruction execution by two the effective address range
 * for the target instruction must be with -126 to +129 bytes
 * of the branch.
 *
 */
uint8_t PZ6502::REL(){
    addrRel = read(pc);
    pc++;

    if(addrRel & 0x80){
        addrRel |= 0xff00;
    }

    return 0;
}


/**
 * Instructions using absolute addressing contain a full 16 bit
 * address to identify the target location.
 */
uint8_t PZ6502::ABS(){
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;

    addrAbs = (hi << 8) | lo;
    return 0;
}


/**
 * Absolute,X
 *
 * The address to be accessed by an instruction using X register
 * indexed absolute addressing is computed by taking the 16 bit address
 * from the instruction and added the contents of the X register.
 * For example if X contains $92 then an STA $2000,X instruction will
 * store the accumulator at $2092 (e.g. $2000 + $92).
 */
uint8_t PZ6502::ABX(){
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;

    addrAbs = (hi << 8) | lo;
    addrAbs += x_reg;

    if((addrAbs & 0xff00) != (hi << 8)){
        return 1;
    }else{
        return 0;
    }
}


/**
 * Absolute,Y
 *
 * The Y register indexed absolute addressing mode is the same
 * as the previous mode only with the contents of the Y register
 * added to the 16 bit address from the instruction.
 */
uint8_t PZ6502::ABY(){
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;

    addrAbs = (hi << 8) | lo;
    addrAbs += y_reg;

    if((addrAbs & 0xff00) != (hi << 8)){
        return 1;
    }else{
        return 0;
    }
}


/**
 * Indirect
 *
 * JMP is the only 6502 instruction to support indirection.
 * The instruction contains a 16 bit address which identifies
 * the location of the least significant byte of another 16 bit memory
 * address which is the real target of the instruction.
 *
 * For example if location $0120 contains $FC and location $0121 contains
 * $BA then the instruction JMP ($0120) will cause the next instruction
 * execution to occur at $BAFC (e.g. the contents of $0120 and $0121).
 */
uint8_t PZ6502::IND(){
    uint16_t ptr_lo = read(pc);
    pc++;
    uint16_t ptr_hi = read(pc);
    pc++;

    uint16_t ptr = (ptr_hi << 8) | ptr_lo;

    if(ptr_lo == 0x00ff){
        addrAbs = read((ptr & 0xff00) << 8) | read(ptr + 0);
    }else{
        addrAbs = (read(ptr + 1) << 8) | read(ptr + 0);
    }

    return 0;
}

/**
 * Indexed Indirect
 *
 * Indexed indirect addressing is normally used in conjunction with
 * a table of address held on zero page. The address of the table is
 * taken from the instruction and the X register added to it (with zero
 * page wrap around) to give the location of the least significant byte
 * of the target address.
 */
uint8_t PZ6502::IZX(){
    uint16_t t = read(pc);
    pc++;

    uint16_t lo = read((uint16_t)(t + (uint16_t)x_reg) & 0x00ff);
    uint16_t hi = read((uint16_t)(t + (uint16_t)x_reg + 1) & 0xff00);

    addrAbs = (hi << 8) | lo;

    return 0;
}

uint8_t PZ6502::IZY(){
    uint16_t t = read(pc);
    pc++;

    uint16_t lo = read(t & 0x00ff);
    uint16_t hi = read((t + 1) & 0x00ff);

    addrAbs = (hi << 8) | lo;
    addrAbs += y_reg;

    if((addrAbs & 0xff00) != (hi << 8)){
        return 1;
    }else{
        return 0;
    }
}


/**
 * Indirect Indexed
 *
 * Indirect indirect addressing is the most common indirection mode
 * used on the 6502. In instruction contains the zero page location
 * of the least significant byte of 16 bit address. The Y register
 * is dynamically added to this value to generated the actual target
 * address for operation.
 */
uint8_t PZ6502::fetch(){
    if(!(m_lookup[opcode].addrmode == &PZ6502::IMP)){
        fetched = read(addrAbs);
    }

    return fetched;
}


/****************************************************************************/
/*                     Opcodes implementations                              */

/**
 * Add Memory to Accumulator with Carry
 * A + M + C -> A, C
 *
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
    fetched = fetch();

    uint16_t temp = (uint16_t)accum + (uint16_t)fetched + (uint16_t)GetFlag(C);
    SetFlag(C, temp > 255);
    SetFlag(Z, (temp & 0x00ff) == 0);
    SetFlag(N, temp & 0x80);
    SetFlag(V, ( ((uint16_t)accum ^ temp) & ~((uint16_t)accum ^ (uint16_t)fetched) ) );

    accum = temp & 0x00ff;
    return 1;
}

//AND Memory with Accumulator
// A AND M -> A
uint8_t PZ6502::AND(){
    fetched = fetch();

    accum = accum & fetched;

    SetFlag(Z, accum == 0x00);
    SetFlag(N, accum & 0x80);

    return 1;
}

// Arithmetic shift left
uint8_t PZ6502::ASL(){
    fetched = fetch();

    uint16_t temp = (uint16_t)fetched << 1;
    SetFlag(C, (temp & 0xff00) > 0);
    SetFlag(Z, (temp & 0x00ff) == 0);
    SetFlag(N, temp & 0x80);

    if (m_lookup[opcode].addrmode == &PZ6502::IMP){
        accum = temp & 0x00ff;
    } else {
        write(addrAbs, temp & 0x00ff);
    }

    return 0;
}

// branch on carry clear
uint8_t PZ6502::BCC(){
    if(GetFlag(C) == 0){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}


// Branch on Carry Set
// branch on C = 1
uint8_t PZ6502::BCS(){
    if(GetFlag(C) == 1){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}


// branch on equal (zero set)
uint8_t PZ6502::BEQ(){
    if(GetFlag(Z) == 1){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}


// Test Bits in Memory with Accumulator
uint8_t PZ6502::BIT(){
    fetched = fetch();

    uint8_t temp = accum & fetched;

    SetFlag(N, (fetched & 0x40));
    SetFlag(V, (fetched & 0x20));
    SetFlag(Z, (temp == 0x00));

    return 0;
}


// branch on minus (negative set)
uint8_t PZ6502::BMI(){
    if(GetFlag(N) == 1){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}



// branch on not equal (zero clear)
uint8_t PZ6502::BNE(){
    if(GetFlag(Z) == 0){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}


// branch on plus (negative clear)
uint8_t PZ6502::BPL(){
    if(GetFlag(N) == 0){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}

// Force Break
// similar to irq
// reference: https://www.pagetable.com/?p=410
uint8_t PZ6502::BRK(){
    pc++;

    SetFlag(I, 1);
    write(0x0100 + stkp, (pc >> 8) && 0x00ff);
    stkp--;
    write(0x0100 + stkp, pc & 0x00ff);
    stkp--;

    SetFlag(B, 1);
    write(0x0100 + stkp, status);
    stkp--;

    // SetFlag(B, 0); // test without this line

    //             hi bytes                     lo bytes
    pc = (uint16_t)(read(0xffff) << 8) | (uint16_t)(read(0xfffe));

    return 0;
}


// Branch on Overflow Clear
uint8_t PZ6502::BVC(){
    if(GetFlag(V) == 0){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}


// Branch on Overflow Set
uint8_t PZ6502::BVS(){
    if(GetFlag(V) == 1){
        cycles++;
        addrAbs = pc + addrRel;

        if((addrAbs & 0xff00) != (pc & 0xff00)){
            cycles++;
        }

        pc = addrAbs;
    }

    return 0;
}


// Clear Carry Flag
uint8_t PZ6502::CLC(){
    SetFlag(C, 0);
    return 0;
}


// Clear Decimal Mode
uint8_t PZ6502::CLD(){
    SetFlag(D, 0);
    return 0;
}


//Clear Interrupt Disable Bit
uint8_t PZ6502::CLI(){
    SetFlag(I, 0);
    return 0;
}

// Clear Overflow Flag
uint8_t PZ6502::CLV(){
    SetFlag(V, 0);
    return 0;
}

// Compare Memory with Accumulator
// reference: 6502.org/tutorials/compare_instructions.html
uint8_t PZ6502::CMP(){
    fetched = fetch();

    uint16_t temp = (uint16_t)accum - (uint16_t)fetched;

    SetFlag(N, temp & 0x0080);
    SetFlag(Z, accum == fetched);
    SetFlag(C, (accum >= fetched));

    // according to the dataset, CMP can potential require additional cycle
    return 1;
}

// Compare Memory with x register
uint8_t PZ6502::CPX(){
    fetched = fetch();

    uint16_t temp = (uint16_t)x_reg - (uint16_t)fetched;

    SetFlag(N, temp & 0x0080);
    SetFlag(Z, x_reg == fetched);
    SetFlag(C, (x_reg >= fetched));

    return 0;
}


// Compare Memory with x register
uint8_t PZ6502::CPY(){
    fetched = fetch();

    uint16_t temp = (uint16_t)y_reg - (uint16_t)fetched;

    SetFlag(N, temp & 0x0080);
    SetFlag(Z, y_reg == fetched);
    SetFlag(C, (y_reg >= fetched));

    return 0;
}


// Decrement Memory by One
uint8_t PZ6502::DEC(){
    fetched = fetch();

    uint16_t temp = (uint16_t)fetched - 1;
    write(addrAbs, temp & 0x00ff);

    SetFlag(Z, (temp & 0x00ff) == 0x0000);
    SetFlag(N, temp & 0x0080);

    return 0;
}


// Decrement x register by One
uint8_t PZ6502::DEX(){

    x_reg--;
    SetFlag(Z, x_reg == 0x00);
    SetFlag(N, x_reg & 0x80);

    return 0;
}

// Decrement x register by One
uint8_t PZ6502::DEY(){

    y_reg--;
    SetFlag(Z, y_reg == 0x00);
    SetFlag(N, y_reg & 0x80);

    return 0;
}


// Exclusive-OR Memory with Accumulator
uint8_t PZ6502::EOR(){
    fetched = fetch();

    accum = accum ^ fetched;

    SetFlag(N, accum & 0x80);
    SetFlag(Z, accum == 0x00);

    return 1;
}


// Increment Memory by One
uint8_t PZ6502::INC(){
    fetched = fetch();

    uint16_t temp = (uint16_t)fetched + 1;
    SetFlag(N, temp & 0x0080);
    SetFlag(Z, (temp & 0x00ff) == 0);
    write(addrAbs, temp & 0x00ff);
    return 0;
}


// Increment X register by one
uint8_t PZ6502::INX(){
    x_reg++;
    SetFlag(N, x_reg & 0x80);
    SetFlag(Z, x_reg == 0x00);
    return 0;
}


// Increment X register by one
uint8_t PZ6502::INY(){
    y_reg++;
    SetFlag(N, y_reg & 0x80);
    SetFlag(Z, y_reg == 0x00);
    return 0;
}


/**
 * Sets the program counter to the address specified by the operand.
 */
uint8_t PZ6502::JMP(){
    pc = addrAbs;
    return 0;
}


/**
 * The JSR instruction pushes the address (minus one) of the
 * return point on to the stack and then sets the program counter
 * to the target memory address.
 */
uint8_t PZ6502::JSR(){
    pc--;

    write(0x0100 + stkp, (pc & 0xff00) >> 8);
    stkp--;
    write(0x0100 + stkp, (pc & 0x00ff));
    stkp--;

    pc = addrAbs;

    return 0;
}

/**
 * LDA - Load Accumulator
 * A,Z,N = M
 *
 * Loads a byte of memory into the accumulator setting
 * the zero and negative flags as appropriate.
 */
uint8_t PZ6502::LDA(){
    fetched = fetch();

    accum = fetched;
    SetFlag(Z, accum == 0x00);
    SetFlag(N, accum & 0x80);

    return 1;
}


/**
 * LDX - Load X Register
 * X,Z,N = M
 * Loads a byte of memory into the X register
 * setting the zero and negative flags as appropriate.
 */
uint8_t PZ6502::LDX(){
    fetched = fetch();
    x_reg = fetched;
    SetFlag(Z, x_reg == 0x00);
    SetFlag(N, x_reg & 0x80);

    return 1;
}


/**
 * LDX - Load X Register
 * X,Z,N = M
 * Loads a byte of memory into the X register
 * setting the zero and negative flags as appropriate.
 */
uint8_t PZ6502::LDY(){
    fetched = fetch();
    y_reg = fetched;
    SetFlag(Z, y_reg == 0x00);
    SetFlag(N, y_reg & 0x80);

    return 1;
}


/**
 * LSR - Logical Shift Right
 *
 * A,C,Z,N = A/2 or M,C,Z,N = M/2
 *
 * Each of the bits in A or M is shift one place to the right.
 * The bit that was in bit 0 is shifted into the carry flag. Bit 7 is set to zero.
 */
uint8_t PZ6502::LSR(){
    fetched = fetch();

    SetFlag(C, (fetched & 0x01));

    uint16_t temp = (uint16_t)fetched >> 1;
    SetFlag(Z, (temp & 0x00ff) == 0);
    SetFlag(N, temp & 0x0080);

    if (m_lookup[opcode].addrmode == &PZ6502::IMP){
        // accum = temp & 0x00ff;
        accum = accum >> 1;
    } else {
        write(addrAbs, temp & 0x00ff);
    }

    return 0;
}

/**
 * NOP - No Operation
 * NOP is used to reserve space for future modifications or
 * effectively REM out existing code.
 */
uint8_t PZ6502::NOP(){
    return 0;
}

uint8_t PZ6502::ORA(){
    fetched = fetch();
    accum = accum | fetched;
    SetFlag(N, accum & 0x80);
    SetFlag(Z, accum == 0x00);
    return 1;
}

/**
 *  push accumulator on stack
 *  stack starts from end of 2nd page, aka 0x01ff
 *  when push accumulator on stack, decrement the stack pointer
 */

uint8_t PZ6502::PHA(){
    write(0x0100 + stkp, accum);
    stkp--;
    return 0;
}

/**
 * PHP
 * Push Processor Status on Stack
 */
uint8_t PZ6502::PHP(){
    // alternative: break flag is set to 1 before push
    // write(0x0100 + stkp, status | B | U)
    // SetFlag(B, 0);
    // SetFlag(U, 0);
    write(0x0100 + stkp, status);
    stkp--;
    return 0;
}

/**
 * popping data from the stack
 * increment the pointer before reading the data
 */
uint8_t PZ6502::PLA(){
    stkp++;
    accum = read(0x0100 + stkp);
    SetFlag(Z, accum == 0x00);
    SetFlag(N, accum & 0x80);
    return 0;
}


/**
 * PuLl Processor status
 * Pulls an 8 bit value from the stack and into the processor flags.
 * The flags will take on new states as determined by the value pulled.
 */
uint8_t PZ6502::PLP(){
    stkp++;
    status = read(0x0100 + stkp);
    return 0;
}

/**
 * ROL shifts all bits left one position.
 * The Carry is shifted into bit 0 and the original bit 7 is
 * shifted into the Carry.
 *
 * Affect flags: N Z C
 */
uint8_t PZ6502::ROL(){
    fetched = fetch();

    uint16_t temp = (uint16_t)(fetched << 1) | GetFlag(C);

    SetFlag(C, temp & 0x0080);
    SetFlag(Z, (temp & 0x00ff) == 0x0000);
    SetFlag(N, temp & 0x0080);

    if(m_lookup[opcode].addrmode == &PZ6502::IMP){
        accum = temp & 0x00ff;
    }else{
        write(addrAbs, temp & 0x00ff);
    }
    return 0;
}


/**
 * Move each of the bits in either A or M one place to the right.
 * Bit 7 is filled with the current value of the carry flag
 * whilst the old bit 0 becomes the new carry flag value.
 *
 * Affect flags: N Z C
 */
uint8_t PZ6502::ROR(){
    fetched = fetch();

    uint16_t temp = (uint16_t)(fetched >> 1) | (GetFlag(C) << 7);

    SetFlag(C, fetched & 0x01);
    SetFlag(Z, (temp & 0x00ff) == 0x0000);
    SetFlag(N, temp & 0x0080);

    if(m_lookup[opcode].addrmode == &PZ6502::IMP){
        accum = temp & 0x00ff;
    }else{
        write(addrAbs, temp & 0x00ff);
    }
    return 0;
}

// Return from Interrupt
uint8_t PZ6502::RTI(){
    stkp++;

    status = read(0x0100 + stkp);
    SetFlag(B, 0);
    SetFlag(U, 0);

    stkp++;
    uint16_t lo = read(0x0100 + stkp);
    stkp++;
    uint16_t hi = read(0x0100 + stkp);

    pc = (hi << 8) | lo;

    return 0;
}

uint8_t PZ6502::RTS(){
    return 0;
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
    fetched = fetch();

    // invert the data
    uint16_t inverted_M = ((uint16_t)fetched) ^ 0x00ff;

    uint16_t temp = (uint16_t)accum + inverted_M + (uint16_t)GetFlag(C);
    SetFlag(C, temp & 0xff00);
    SetFlag(Z, (temp & 0x00ff) == 0);
    SetFlag(N, temp & 0x80);
    SetFlag(V, ( ((uint16_t)accum ^ temp) & ~((uint16_t)accum ^ inverted_M) ) );

    accum = temp & 0x00ff;
    return 1;
}

uint8_t PZ6502::SEC(){
    SetFlag(C, 1);
    return 0;
}

uint8_t PZ6502::SED(){
    SetFlag(D, 1);
    return 0;
}

uint8_t PZ6502::SEI(){
    SetFlag(I, 1);
    return 0;
}

uint8_t PZ6502::STA(){
    write(addrAbs, accum);
    return 0;
}

uint8_t PZ6502::STX(){
    write(addrAbs, x_reg);
    return 0;
}

uint8_t PZ6502::STY(){
    write(addrAbs, y_reg);
    return 0;
}

uint8_t PZ6502::TAX(){
    x_reg = accum;
    SetFlag(N, x_reg & 0x80);
    SetFlag(Z, x_reg == 0);
    return 0;
}

uint8_t PZ6502::TAY(){
    y_reg = accum;
    SetFlag(N, y_reg & 0x80);
    SetFlag(Z, y_reg == 0x00);
    return 0;
}

uint8_t PZ6502::TSX(){
    x_reg = stkp;
    SetFlag(N, x_reg & 0x80);
    SetFlag(Z, x_reg == 0x00);
    return 0;
}

uint8_t PZ6502::TXA(){
    accum = x_reg;
    SetFlag(N, accum & 0x80);
    SetFlag(Z, accum == 0x00);
    return 0;
}

uint8_t PZ6502::TXS(){
    stkp = x_reg;
    return 0;
}

uint8_t PZ6502::TYA(){
    accum = y_reg;
    SetFlag(N, accum & 0x80);
    SetFlag(Z, accum == 0x00);
    return 0;
}

// this function captures illegal opcodes
uint8_t PZ6502::XXX(){
    return 0;
}


void PZ6502::reset(){


    addrAbs = 0xfffc; // go to reset instruction in memory
    uint16_t lo = read(addrAbs + 0);
    uint16_t hi = read(addrAbs + 1);

    pc = (hi << 8) | lo;

    accum = 0x00;
    x_reg = 0x00;
    x_reg = 0x00;
    stkp = 0xff;
    status = 0x00;

    addrAbs = 0x0000;
    addrRel = 0x0000;
    fetched = 0x00;

    cycles = 8;
}

/**
 * At the occurrence of interrupt, the value of the program counter (PC) is put in
 * high-low order onto the stack, followed by the value currently in the status
 * register and control will be transferred to the address location found in the
 * respective interrupt vector. These are recovered from the stack at the end of
 * an interrupt routine by the RTI instruction.
*/

void PZ6502::irq(){
    if(GetFlag(I) == 0){
        write(0x0100 + stkp, (pc >> 8) & 0x00ff);
        stkp--;
        write(0x0100 + stkp, pc & 0x00ff);
        stkp--;

        SetFlag(B, 0);
        SetFlag(I, 1);
        SetFlag(U, 1);
        write(0x0100 + stkp, status);
        stkp--;

        addrAbs = 0xfffe;
        uint16_t lo = read(addrAbs + 0);
        uint16_t hi = read(addrAbs + 1);

        pc = (hi << 8) | lo;

        cycles = 7;
    }
}

/**
 * Similar to IRQ except nothing can stop NMI from happening
  */
void PZ6502::nmi(){
    write(0x0100 + stkp, (pc >> 8) & 0x00ff);
    stkp--;
    write(0x0100 + stkp, pc & 0x00ff);
    stkp--;

    SetFlag(B, 0);
    SetFlag(I, 1);
    SetFlag(U, 1);
    write(0x0100 + stkp, status);
    stkp--;

    addrAbs = 0xfffa;
    uint16_t lo = read(addrAbs + 0);
    uint16_t hi = read(addrAbs + 1);

    pc = (hi << 8) | lo;

    cycles = 7;
}

bool PZ6502::complete(){
    return cycles == 0;
}


// This is the disassembly function. Its workings are not required for emulation.
// It is merely a convenience function to turn the binary instruction code into
// human readable form. Its included as part of the emulator because it can take
// advantage of many of the CPUs internal operations to do this.
std::map<uint16_t, std::string> PZ6502::disassemble(uint16_t nStart, uint16_t nStop)
{
	uint32_t addr = nStart;
	uint8_t value = 0x00, lo = 0x00, hi = 0x00;
	std::map<uint16_t, std::string> mapLines;
	uint16_t line_addr = 0;

	// A convenient utility to convert variables into
	// hex strings because "modern C++"'s method with
	// streams is atrocious
	auto hex = [](uint32_t n, uint8_t d)
	{
		std::string s(d, '0');
		for (int i = d - 1; i >= 0; i--, n >>= 4)
			s[i] = "0123456789ABCDEF"[n & 0xF];
		return s;
	};

	// Starting at the specified address we read an instruction
	// byte, which in turn yields information from the lookup table
	// as to how many additional bytes we need to read and what the
	// addressing mode is. I need this info to assemble human readable
	// syntax, which is different depending upon the addressing mode

	// As the instruction is decoded, a std::string is assembled
	// with the readable output
	while (addr <= (uint32_t)nStop)
	{
		line_addr = addr;

		// Prefix line with instruction address
		std::string sInst = "$" + hex(addr, 4) + ": ";

		// Read instruction, and get its readable name
		uint8_t opcode = m_bus->read(addr, true); addr++;
		sInst += m_lookup[opcode].name + " ";

		// Get oprands from desired locations, and form the
		// instruction based upon its addressing mode. These
		// routines mimmick the actual fetch routine of the
		// 6502 in order to get accurate data as part of the
		// instruction
		if (m_lookup[opcode].addrmode == &PZ6502::IMP)
		{
			sInst += " {IMP}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::IMM)
		{
			value = m_bus->read(addr, true); addr++;
			sInst += "#$" + hex(value, 2) + " {IMM}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::ZP0)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "$" + hex(lo, 2) + " {ZP0}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::ZPX)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "$" + hex(lo, 2) + ", X {ZPX}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::ZPY)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "$" + hex(lo, 2) + ", Y {ZPY}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::IZX)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "($" + hex(lo, 2) + ", X) {IZX}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::IZY)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = 0x00;
			sInst += "($" + hex(lo, 2) + "), Y {IZY}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::ABS)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = m_bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + " {ABS}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::ABX)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = m_bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", X {ABX}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::ABY)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = m_bus->read(addr, true); addr++;
			sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", Y {ABY}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::IND)
		{
			lo = m_bus->read(addr, true); addr++;
			hi = m_bus->read(addr, true); addr++;
			sInst += "($" + hex((uint16_t)(hi << 8) | lo, 4) + ") {IND}";
		}
		else if (m_lookup[opcode].addrmode == &PZ6502::REL)
		{
			value = m_bus->read(addr, true); addr++;
			sInst += "$" + hex(value, 2) + " [$" + hex(addr + value, 4) + "] {REL}";
		}

		// Add the formed string to a std::map, using the instruction's
		// address as the key. This makes it convenient to look for later
		// as the instructions are variable in length, so a straight up
		// incremental index is not sufficient.
		mapLines[line_addr] = sInst;
	}

	return mapLines;
}