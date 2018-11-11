/*
   thingdev.hpp : Board subclass for prototyping on Sparkfun ESP8266 ThingDev board

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

#include "hackflight.hpp"
#include "realboard.hpp"

namespace hf {

    class SparkfunEsp8266ThingDev : public RealBoard {

        private:

            static const uint8_t LED_PIN = 5;

        protected:

            void delaySeconds(float sec)
            {
                delay((uint32_t)(1000*sec));
            }

            void setLed(bool isOn)
            { 
                digitalWrite(LED_PIN, isOn ? LOW : HIGH);
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

            virtual uint32_t getMicroseconds(void) override
            {
                return micros();
            }

            virtual bool getQuaternion(float quat[4]) override
            {
                (void)quat;
                return false;
            }

            virtual bool getGyrometer(float gyroRates[3]) override
            {
                (void)gyroRates;
                return false;
            }

            virtual void writeMotor(uint8_t index, float value) override
            {
                (void)index;
                (void)value;
            }

        public:

            SparkfunEsp8266ThingDev(void)
            {
                pinMode(LED_PIN, OUTPUT);
                Serial.begin(115200);
            }

    }; // class SparkfunEsp8266ThingDev

    void Board::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace hf
