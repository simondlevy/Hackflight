#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <time.h>

class TwoWire {
    
    public:

        void begin();

        void beginTransmission(const uint8_t addr);

        uint8_t endTransmission(const uint8_t stop=0);

        uint8_t read();

        size_t requestFrom(
                const uint8_t address, const size_t size, bool sendStop=0);

        void write(const uint8_t value);

    private:

        uint8_t _buffer[256];

        uint8_t _bufidx;

        uint8_t _addr;
};
