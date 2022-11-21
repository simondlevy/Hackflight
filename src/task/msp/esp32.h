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
#include <string.h>

#include <esp_now.h>

#include "task/msp.h"

class Esp32Msp : public Msp {

    private:

        uint8_t m_receiverAddress[6];

    protected:

        virtual void serialBegin(const uint32_t baud) override
        {
        }

        virtual uint32_t serialAvailable(void) override
        {
            return 0;
        }

        virtual uint8_t serialRead(void) override
        {
            return 0;
        }

        virtual void serialWrite(const uint8_t buf[], const uint8_t count) override
        {
            esp_now_send(m_receiverAddress, buf, count);
        }

    public:

            Esp32Msp(const uint8_t receiverAddress[6])
            {
                memcpy(m_receiverAddress, receiverAddress, 6);
            }

}; // class Esp32Msp
