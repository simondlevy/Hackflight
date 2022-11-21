/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "task/msp.h"

class UartMsp : public Msp {

    private:

        HardwareSerial * m_port;

    protected:

        virtual void serialBegin(uint32_t baud) override
        {
            m_port->begin(baud);
        }

        virtual uint32_t serialAvailable(void) override
        {
            return m_port->available();
        }

        virtual uint8_t serialRead(void) override
        {
            return m_port->read();
        }

        virtual void serialWrite(uint8_t buf[], uint8_t count) override
        {
            m_port->write(buf, count);
        }

    public:

        UartMsp(HardwareSerial & port)
        {
            m_port = &port;
        }

}; // class UartMsp
