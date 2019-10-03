/*
   Board subclass for ESP32 Feather

   Copyright (c) 2019 Simon D. Levy

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
#include "motors/standard.hpp"

namespace hf {

    class ESP32FeatherBoard : public SentralBoard {

        private:

            StandardMotor motors[4] = { 
                StandardMotor(33), 
                StandardMotor(27), 
                StandardMotor(18), 
                StandardMotor(5) 
            };

        protected:

            virtual void adjustQuaternion(float & qw, float & qx, float & qy, float & qz) override
            { 
                Filter::swap(qw, qx);
                Filter::swap(qy, qz);

                qx = -qx;
                qy = -qy;
            }

            virtual void writeMotor(uint8_t index, float value) override
            {
                motors[index].write(value);
            }

        public:

            ESP32FeatherBoard(void) 
                : SentralBoard(13)
            {

                // Use D32, D15 for power
                powerPin(32, HIGH);
                powerPin(15, LOW);

                // Hang a bit
                delay(100);

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);

                // Start the USFS
                SentralBoard::begin();

                // Initialize the motors
                for (uint8_t k=0; k<4; ++k) {
                    motors[k].init();
                }

                // Hang a bit more
                delay(100);
            }

    }; // class ESP32Feather

} // namespace hf
