#include "Bus.h"

Bus::Bus(){
    // clearing RAM contents
    for(auto &i: ram){
        i = 0x00;
    }

    // connect cpu to communication bus
    cpu.ConnectBus(this);
}

Bus::~Bus(){
}

void Bus::write(uint16_t addr, uint8_t data){

}

uint8_t Bus::read(uint16_t addr, bool bReadOnly=false){

}