/*
   Board subclass for Arduino prototyping without IMU or motors

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

#include "boards/arduino/arduino.hpp"

namespace hf {

    class MockBoard : public ArduinoBoard {

        private:

            float _time = 0;

            // motionless and sligthly off-level
            float _qw = 0.90f;
            float _qx = 0.05f;
            float _qy = 0.05f;
            float _qz = 0.00f;
            float _gx = 0.00f;
            float _gy = 0.00f;
            float _gz = 0.00f;

        protected:

            virtual bool  getQuaternion(float & qw, float & qx, float & qy, float & qz) override 
            {
                qw = _qw;
                qx = _qx;
                qy = _qy;
                qz = _qz;

                float time = getTime();

                if (time-_time > .01) {
                    _time = time;
                    return true;
                }

                return false;
            }

            virtual bool getGyrometer(float & gx, float & gy, float & gz) override
            {
                gx = _gx;
                gy = _gy;
                gz = _gz;

                float time = getTime();

                if (time-_time > .01) {
                    _time = time;
                    return true;
                }

                return false;
             }

            uint8_t serialTelemetryAvailable(void) override
            {
                return Serial1.available();
            }

            uint8_t serialTelemetryRead(void) override
            {
                return Serial1.read();
            }

            void serialTelemetryWrite(uint8_t c) override
            {
                Serial1.write(c);
            }

        public:

            MockBoard(uint8_t ledPin, bool ledInverted=false) 
                : ArduinoBoard(ledPin, ledInverted)
            {

                // Set up to receive telemetry over Serial1
                Serial1.begin(115200);
                _time = 0;
            }

    }; // class MockBoard

} // namespace hf
