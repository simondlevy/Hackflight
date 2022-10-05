/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "arming.h"
#include "core/axes.h"
#include "core/clock.h"
#include "core/constrain.h"
#include "core/filters/pt1.h"
#include "core/vstate.h"
#include "imu.h"
#include "stats.h"
#include "system.h"
#include "time.h"

#include "serial.h"

class Imu {

    friend class Hackflight;
    friend class Task;
    friend class AttitudeTask;
    friend class Receiver;

    protected:

        typedef void (*align_fun)(Axes * axes);

        virtual void begin(void) = 0;

        virtual auto getEulerAngles(const uint32_t time) -> Axes = 0;

        virtual uint32_t getGyroInterruptCount(void) = 0;

        virtual int32_t getGyroSkew(
                const uint32_t nextTargetCycles, const int32_t desiredPeriodCycles) = 0;

        virtual bool gyroIsCalibrating(void) = 0;

        virtual bool gyroIsReady(void) = 0;

        virtual auto readGyroDps(const align_fun align) -> Axes = 0;

}; // class Imu
