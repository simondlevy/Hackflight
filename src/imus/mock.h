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

#include "imu.h"

class MockImu : public Imu {

    protected:

        virtual bool devGyroIsReady(void) override
        {
            return false;
        }

        virtual void devInit(
                uint32_t * gyroSyncTimePtr,
                uint32_t * gyroInterruptCountPtr) override
        {
            (void)gyroSyncTimePtr;
            (void)gyroInterruptCountPtr;
        }

        virtual int16_t devReadRawGyro(uint8_t k) override
        {
            (void)k;
            return 0;
        }

    public:

        MockImu(void)
            : Imu(0)
        {
        }

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            (void)time;
            return Axes(0.1, 0.1, 0.1); // simulate tilt
        }
};
