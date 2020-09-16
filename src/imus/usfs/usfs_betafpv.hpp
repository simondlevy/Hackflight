/*
   Support for USFS IMU on BetaFPV-style flight controller

   Copyright (c) 2020 Simon D. Levy

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

namespace hf {

    class USFS_BetaFPV : public USFS {

        protected:

            virtual bool getQuaternion(float & qw, float & qx, float & qy, float & qz, float time) override
            {
                USFS::getQuaternion(qw, qx, qy, qz, time);
            }

            virtual bool getGyrometer(float & gx, float & gy, float & gz) override
            {
                USFS::getGyrometer(gx, gy, gz);
            }


    }; // class USFS_BetaFPV

} // namespace hf
