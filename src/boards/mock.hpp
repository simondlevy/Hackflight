/*
   mock.hpp : Board subclass for Arduino prototyping without IMU or motors

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

#include "arduino.hpp"

namespace hf {

    class MockBoard : public ArduinoBoard {

        private:

            float _time;

            // level and motionless
            float _quat[4] = {1.f, 0.f, 0.f, 0.f};
            float _gyro[3] = {0.f, 0.f, 0.f};

        protected:

            virtual bool getQuaternion(float quat[4]) override
            {
                memcpy(quat, _quat, 4*sizeof(float));

                float time = getTime();

                if (time-_time > .01) {
                    _time = time;
                    return true;
                }

                return false;
            }

            virtual bool getGyrometer(float gyroRates[3]) override
            {
                memcpy(gyroRates, _gyro, 3*sizeof(float));

                float time = getTime();

                if (time-_time > .01) {
                    _time = time;
                    return true;
                }

                return false;
             }

            virtual void writeMotor(uint8_t index, float value) override
            {
                (void)index;
                (void)value;
            }

        public:

            MockBoard(uint8_t ledPin, bool ledInverted=false) : ArduinoBoard(ledPin, ledInverted)
            {
                _time = 0;
            }

    }; // class MockBoard

} // namespace hf
