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

#include <stdbool.h>
#include <stdint.h>

#include "arming.h"

void failsafeInit(void);
void failsafeReset(void);
void failsafeStartMonitoring(void);
void failsafeUpdateState(float * rcData, void * motorDevice, Arming::data_t * arming);
bool failsafeIsMonitoring(void);
bool failsafeIsActive(void);
void failsafeOnValidDataReceived(Arming::data_t * arming);
void failsafeOnValidDataFailed(Arming::data_t * arming);

class Failsafe {

    public:

        void init(void)
        {
        }

        void reset(void)
        {
        }

        void startMonitoring(void)
        {
        }

        void updateState(
                float * rcData, void * motorDevice, Arming::data_t * arming)
        {
            (void)rcData;
            (void)motorDevice;
            (void)arming;
        }

        bool isMonitoring(void)
        {
            return false;
        }

        bool isActive(void)
        {
            return false;
        }

        void onValidDataReceived(Arming::data_t * arming)
        {
            (void)arming;
        }

        void onValidDataFailed(Arming::data_t * arming)
        {
            (void)arming;
        }
};
