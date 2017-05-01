/*
   imu.hpp : IMU class header

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/imu.c

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

namespace hf {

class IMU {
private:

public:

    void init(void);
    void update(uint32_t currentTime, bool armed);
};


/********************************************* CPP ********************************************************/

void IMU::init(void)
{
}

void IMU::update(uint32_t currentTime, bool armed)
{
    (void)currentTime;
    (void)armed;
}

} // namespace hf

