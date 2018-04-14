/*
   butterfly.hpp : Ladybug dev board implementation of Hackflight Board routines

   Uses MPU9250

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
#include <MPU9250.h>
#include <stdarg.h>
#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class Butterfly : public RealBoard {

        private:

            MPU9250 * mpu9250;
            const uint8_t LED = 25;

        protected:

            void init(void)
            {
                // Create the MPU9250 object
                mpu9250 = new MPU9250(Wire, 0x68);

                // start communication with MPU9250 
                int status = mpu9250->begin();
                if (status < 0) {
                    Serial.println("IMU initialization unsuccessful");
                    Serial.println("Check IMU wiring or try cycling power");
                    Serial.print("Status: ");
                    Serial.println(status);
                    while(true) ;
                }

                // Begin serial comms
                Serial.begin(115200);

                // Setup LEDs and turn them off
                pinMode(LED, OUTPUT);
                digitalWrite(LED, LOW);

                // Start I^2C
                Wire.begin();

                // Hang a bit before starting up the EM7180
                delay(100);

                // Do general real-board initialization
                RealBoard::init();
            }

            void delayMilliseconds(uint32_t msec)
            {
                delay(msec);
            }

            void ledSet(bool is_on)
            { 
                digitalWrite(LED, is_on ? HIGH : LOW);
            }

            uint8_t serialAvailableBytes(void)
            {
                return Serial.available();
            }

            uint8_t serialReadByte(void)
            {
                return Serial.read();
            }

            void serialWriteByte(uint8_t c)
            {
                Serial.write(c);
            }

            void writeMotor(uint8_t index, float value)
            {
            }

            bool getGyroRates(float gyroRates[3])
            {
                return false;
            }

            bool getQuaternion(float quat[4])
            {
                return false;
            }

            bool getAccelerometer(float accelGs[3])
            {
                return false;
            }

            bool getBarometer(float & pressure)
            {
                return false;
            }

    }; // class Butterfly

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
