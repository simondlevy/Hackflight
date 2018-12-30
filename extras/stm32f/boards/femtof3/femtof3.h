/*
   f3evobrushed.h : Board class for Hyperion F3 Evo Brushed board

   Copyright (C) 2018 Simon D. Levy 

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

#include <MPU6500.h>
#include "stm32fboard.h"

class FemtoF3 : public Stm32FBoard {

    private:

        MPU6500 * _imu;

    protected: 

        // SoftwareQuaternionBoard class overrides
        virtual bool imuReady(void) override;
        virtual void imuReadAccelGyro(void) override;

    public:

        FemtoF3(void);

}; // class FemtoF3
