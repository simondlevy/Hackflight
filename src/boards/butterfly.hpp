/*
   butterfly.hpp : Butterfly Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2018 Simon D. Levy

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

#pragma once

#include <Wire.h>
#include "sentral.hpp"

namespace hf {

    class Butterfly : public SentralBoard {


        private:

            const uint8_t MOTOR_PINS[4] = {5, 8, 9, 11};

            static void powerPin(uint8_t id, uint8_t value)
            {
                pinMode(id, OUTPUT);
                digitalWrite(id, value);
            }

            // Min, max PWM values
            const uint16_t PWM_MIN = 1000;
            const uint16_t PWM_MAX = 2000;

         protected:

            virtual void writeMotor(uint8_t index, float value) override
            {
                analogWrite(MOTOR_PINS[index], (uint16_t)(PWM_MIN+value*(PWM_MAX-PWM_MIN)) >> 3);
            }

        public:

            Butterfly(void) : SentralBoard(13, true) // red LED, active low
            {
                // User D4 for power, D3 for ground
                powerPin(4, HIGH);
                powerPin(3, LOW);

                // Hang a bit 
                delay(100);

                // Start I^2C
                Wire.begin(TWI_PINS_6_7);

                // Hang a bit
                delay(100);

                SentralBoard::begin();

                // Initialize the motors

                // Hang a bit more
                delay(100);
            }

    }; // class Butterfly

} // namespace hf
