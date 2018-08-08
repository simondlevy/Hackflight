/*
Wire.h : Arduino-style API for SMT32F boards

Copyright (C) 2018 Simon D. Levy 

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Hackflight is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
*/

extern "C" {

#include <stdint.h>

class HardwareWire {

    private:

        uint8_t addr;
        uint8_t reg;
        uint8_t data;
        uint8_t buffer[256];
        uint8_t bufpos;
        uint8_t avail;

    public:

        void     begin(uint8_t bus);

        void     beginTransmission(uint8_t address);

        uint8_t  write(uint8_t data);

        uint8_t  requestFrom(uint8_t address, uint8_t len);

        uint8_t  available(void);

        uint8_t  read(void);

        uint8_t  endTransmission(bool stop=true);
};

} // extern "C"

extern HardwareWire Wire;
