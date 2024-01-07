/**
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <datatypes.h>

class Mixer {

    public:

        void init(void);

        void run(const demands_t & demands, float motors[]);

    protected:

        static const uint8_t MAX_MOTOR_COUNT = 20;

        float _uncapped[MAX_MOTOR_COUNT];

        void init(const uint8_t motorCount)
        {
            _motorCount = motorCount;
        }

        void cap(float capped[])
        {
            const float maxAllowedThrust = UINT16_MAX;

            // Find highest thrust
            float highestThrustFound = 0;
            for (uint8_t k=0; k<_motorCount; k++) {

                const auto thrust = _uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > maxAllowedThrust) {
                reduction = highestThrustFound - maxAllowedThrust;
            }

            for (uint8_t k = 0; k < _motorCount; k++) {
                float thrustCappedUpper = _uncapped[k] - reduction;
                capped[k] = capMinThrust(thrustCappedUpper);
            }
        }

    private:

        static uint16_t capMinThrust(float thrust) 
        {
            return thrust < 0 ? 0 : thrust;
        }

        uint8_t _motorCount; // needed for inheritance of virtual methods
};
