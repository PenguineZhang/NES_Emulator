#include "Bus.h"
#include "PZ6502.h"

PZ6502::PZ6502(){
    using pz = PZ6502;

    // TODO: INSTRUCTIONS lookup table here
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

void PZ6502::SetFlag(FLAGS6502 f, bool v){

}