/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#include <tasks/estimator.hpp>

class ZRanger {

    public:

        void init()
        {
            // pre-compute constant in the measurement noise model for kalman
            _expCoeff =
				logf(EXP_STD_B / EXP_STD_A) / (EXP_POINT_B - EXP_POINT_A);

            device_init();
        }

        bool read(tofMeasurement_t & tofData)
        {
            float range = device_read();

            // check if range is feasible and push into the estimator the
            // sensor should not be able to measure >5 [m], and outliers
            // typically occur as >8 [m] measurements
            if (range < OUTLIER_LIMIT_MM) {

                float distance = range / 1000; // Scale from [mm] to [m]

                float stdDev = EXP_STD_A * (
                        1 + expf(_expCoeff * (distance - EXP_POINT_A)));

                tofData.timestamp = xTaskGetTickCount();
                tofData.distance = distance;
                tofData.stdDev = stdDev;

                return true;
            }

            return false;
        }

    private:

        static constexpr float FREQ_HZ = 40;

        static const uint16_t OUTLIER_LIMIT_MM = 5000;

        // Measurement noise model
        static constexpr float EXP_POINT_A = 2.5;
        static constexpr float EXP_STD_A = 0.0025; 
        static constexpr float EXP_POINT_B = 4.0;
        static constexpr float EXP_STD_B = 0.2;   

        float _expCoeff;

        bool device_init();

        float device_read();
};
