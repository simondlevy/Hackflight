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

#include "datatypes.h"

class Mixer {

    public:

        void init(void);

        void run(const demands_t & demands, float motors[]);

        void capMotors(const float uncapped[], uint16_t capped[])
        {
            const int32_t maxAllowedThrust = UINT16_MAX;

            // Find highest thrust
            int32_t highestThrustFound = 0;
            for (int motorIndex = 0; motorIndex < _motorCount; motorIndex++) {

                const auto thrust = uncapped[motorIndex];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            int32_t reduction = 0;

            if (highestThrustFound > maxAllowedThrust) {
                reduction = highestThrustFound - maxAllowedThrust;
            }

            for (int motorIndex = 0; motorIndex < _motorCount; motorIndex++) {
                int32_t thrustCappedUpper = uncapped[motorIndex] - reduction;
                capped[motorIndex] = capMinThrust(thrustCappedUpper);
            }
        }

    protected:

        void init(const uint8_t motorCount)
        {
            _motorCount = motorCount;
        }

    private:

        static uint16_t capMinThrust(float thrust) 
        {
            return thrust < 0 ? 0 : thrust;
        }

        uint8_t _motorCount; // needed for inheritance of virtual methods
};
