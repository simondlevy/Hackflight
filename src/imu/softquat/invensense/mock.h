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

class MockImu : public InvenSenseImu {

    protected:

        virtual void getRegisterSettings(
                std::vector<registerSetting_t> & settings) override
        {
            (void)settings;
        }
 
        virtual auto getEulerAngles(const uint32_t time) -> Axes override
        {
            // Simulates rocking in the X (phi) axis

            (void)time;

            static float phi;
            static int8_t dir = +1;

            phi += .01 * dir;

            if (phi >= 1.0) {
                dir = -1;
            }

            if (phi <= -1.0) {
                dir = +1;
            }

            return Axes(phi, 0.1, 0.1);
        }

        virtual int16_t readRawAccel(uint8_t k) override
        {
            return 0;
        }

    public:

        MockImu(void)
            : InvenSenseImu(0, 0, 0, 0)
            {
            }
 };
