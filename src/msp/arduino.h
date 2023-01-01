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

#include "msp.h"

class ArduinoMsp : public Msp {

    public:

        void sendPayload(void) override
        {
            for (auto k=0; k<m_payloadSize; ++k) {
                Serial.write(m_payload[k]);
            }
        }

        void sendPayload(HardwareSerial & uart)
        {
            for (auto k=0; k<m_payloadSize; ++k) {
                uart.write(m_payload[k]);
            }
        }

}; // class Msp
