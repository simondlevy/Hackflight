/*
   Butterfly Flight Controller implementation of Hackflight Board routines

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
#include "arduino.hpp"
#include "sentral.hpp"
#include "motors/standard.hpp"

namespace hf {

    class Butterfly : public ArduinoBoard {

        private:

            StandardMotor motors[4] = { 
                StandardMotor(5), 
                StandardMotor(8), 
                StandardMotor(9), 
                StandardMotor(11) 
            };

            SentralBoard sentral;

         protected:

            virtual bool  getQuaternion(float & qw, float & qx, float & qy, float & qz) override
            {
                return sentral.getQuaternion(qw, qx, qy, qz);
            }

            virtual bool  getGyrometer(float & gx, float & gy, float & gz) override
            {
                return sentral.getGyrometer(gx, gy, gz);
            }
 
            virtual void writeMotor(uint8_t index, float value) override
            {
                motors[index].write(value);
            }

            virtual uint8_t serialTelemetryAvailable(void) override
            {
                return Serial2.available();
            }

            virtual uint8_t serialTelemetryRead(void) override
            {
                return Serial2.read();
            }

            virtual void serialTelemetryWrite(uint8_t c) override
            {
                Serial2.write(c);
            }

         public:

            Butterfly(void) 
                : ArduinoBoard(13, true) // red LED, active low
            {
                // Start telemetry on Serial2
                Serial2.begin(115200);

                // Use D4 for power, D3 for ground
                powerPins(4, 3);

                // Hang a bit 
                delay(100);

                // Start I^2C
                Wire.begin(TWI_PINS_6_7);

                // Hang a bit
                delay(100);

                // Start the USFS
                sentral.begin();

                // Initialize the motors
                for (uint8_t k=0; k<4; ++k) {
                    motors[k].init();
                }

                // Hang a bit more
                delay(100);
            }

    }; // class Butterfly

} // namespace hf
