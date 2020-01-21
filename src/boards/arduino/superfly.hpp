/*
   Superfly Hackable ESP8266 Flight Controller implementation of Hackflight Board routines

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

namespace hf {

    class SuperflyMotor : public Motor {

        public:

            SuperflyMotor(uint8_t pin) 
                : Motor(pin)
            {
            }

        protected:

            virtual void write(float value) override
            {
                analogWrite(_pin, (uint16_t)(value * 1023));
            }

            virtual void init(void) override
            {
                analogWrite(_pin, 0);  
            }

    }; // class SuperflyMotor

    SuperflyMotor motor1(4);
    SuperflyMotor motor2(5);
    SuperflyMotor motor3(12);
    SuperflyMotor motor4(14);

    Motor * superflyMotors[4] = { &motor1, &motor2, &motor3, &motor4 };

    class SuperFly : public ArduinoBoard {

        private:

            const uint8_t MOTOR_PINS[4] = {4, 5, 12, 14};

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

        public:

            SuperFly(void) 
                : ArduinoBoard(15)
            {
                // Start I^2C
                Wire.begin(0,2); // SDA (0), SCL (2) on ESP8266

                // Hang a bit before starting up the EM7180
                delay(100);

                sentral.begin();

                // Initialize the motors
                analogWriteFreq(200);  

                // Hang a bit more
                delay(100);
            }

    }; // class SuperFly

} // namespace hf
