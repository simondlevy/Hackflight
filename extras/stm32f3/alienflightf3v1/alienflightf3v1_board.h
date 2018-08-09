/*
   alienflightf3v1_board.h 

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

#include <f3board.h>

class AlienflightF3V1 : public F3Board {

    bool getGyrometer(float gyroRates[3]) override;

    bool getQuaternion(float quat[4]) override;

    void writeMotor(uint8_t index, float value) override;

    uint8_t serialAvailableBytes(void);

    uint8_t serialReadByte(void);

    void serialWriteByte(uint8_t c);
};
