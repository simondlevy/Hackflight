/*
   arduinosentral.hpp : parent class for Arduino boards using
   EM7180 SENtral Sensor Fusion Solution

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
#include "sentral.hpp"

namespace hf {

    class ArduinoSentralBoard : public ArduinoBoard, public SentralBoard {

        protected:

            ArduinoSentralBoard(uint8_t ledPin, bool ledInverted=false) : ArduinoBoard(ledPin, ledInverted)
            {
            }

            bool getGyrometer(float gyro[3])
            {
                return SentralBoard::getGyrometer(gyro);
            }

            bool getQuaternion(float quat[4])
            {
                return SentralBoard::getQuaternion(quat);
            }

    }; // class ArduinoSentralBoard

} // namespace hf
