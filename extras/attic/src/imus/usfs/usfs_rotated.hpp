/*
   Support for USFS IMU roated by 90 degrees

   Copyright (c) 2021 Simon D. Levy

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

#include "imus/usfs.hpp"
#include "filters.hpp"

namespace hf {

    class USFS_Rotated : public USFS {

        protected:

            virtual void adjustGyrometer(float & x, float & y, float & z) override
            { 
                float tmp = x;
                x = y;
                y = -tmp;
            }

            virtual void adjustQuaternion(float & w, float & x, float & y, float & z) override
            {
                float tmp = x;
                x = -y;
                y = tmp;
            }

            virtual void adjustEulerAngles(float & x, float & y, float & z) override
            {
                x = -x;
                y = -y;
            }

    }; // class USFS_Rotated

} // namespace hf
