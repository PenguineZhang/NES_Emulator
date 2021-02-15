#include "Bus.h"

Bus::Bus(){
    // clearing RAM contents
    for(auto &i: ram){
        i = 0x00;
    }
}

Bus::~Bus(){
}

void Bus::write(uint16_t addr, uint8_t data){

}