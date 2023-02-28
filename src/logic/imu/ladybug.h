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

#include "logic/imu.h"

class LadybugImu : public Imu {

    friend class LadybugBoard;

    private:

        static const uint16_t GYRO_SCALE_DPS  = 2000;

        float qw;
        float qx;
        float qy;
        float qz;

        bool gotNewData;

        Usfs usfs;

        LadybugImu(void) 
            : Imu(Imu::rotate0, GYRO_SCALE_DPS)
        {
        }

        void begin(const uint32_t clockSpeed) override
        {
            (void)clockSpeed;

            setGyroCalibrationCycles();
        }

    public:

        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            (void)time;

            Axes angles = quat2euler(qw, qx, qy, qz);

            return Axes(angles.x, -angles.y, -angles.z);
        }

        virtual void handleInterrupt(const uint32_t cycleCounter) override
        {
            (void)cycleCounter;

            gotNewData = true;
        }
};
