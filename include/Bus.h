#ifndef BUS_H__
#define BUS_H__

#include "PZ6502.h"

#include <cstdint>
#include <iostream>
#include <array>


class Bus{
    public:
        Bus();
        ~Bus();

        //! Devices on the bus
        // cpu
        PZ6502 cpu;

        // RAM (64 bytes)
        std::array<uint8_t, 64 * 1024> ram;

    public:
        void write(uint16_t addr, uint8_t data);
        uint8_t read(uint16_t addr, bool bReadOnly=false);
};

#endif //BUS_H__