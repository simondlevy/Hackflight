/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

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
#include "motors/brushed.hpp"

namespace hf {

    BrushedMotor motor1(13);
    BrushedMotor motor2(A2);
    BrushedMotor motor3(3);
    BrushedMotor motor4(11);

    Motor * ladybugFcMotors[4] = { &motor1, &motor2, &motor3, &motor4 };

    class LadybugFC : public ArduinoBoard {

        private:

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

            // Support prototype version where LED is on pin A1
            LadybugFC(uint8_t ledPin = A4) 
                : ArduinoBoard(ledPin)
            {
                // Start I^2C
                Wire.begin();

                // Hang a bit before starting up the EM7180
                delay(100);

                sentral.begin();

                // Hang a bit more
                delay(100);
            }

    }; // class LadybugFC

} // namespace hf
