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

#include <stdbool.h>

#include "datatypes.h"

#if defined(__cplusplus)
extern "C" {
#endif

void mspInit(void);

void mspTriggerDebugging(void);

void mspUpdate(
        vehicleState_t * state,
        rx_axes_t *rxaxes,
        bool armed,
        void * motorDevice,
        float * motors);

#if defined(__cplusplus)
}
#endif

