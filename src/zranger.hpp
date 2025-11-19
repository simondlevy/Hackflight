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

#include <VL53L1X.h>

#include <debugger.hpp>
#include <ekf.hpp>

class ZRanger {

    public:

        void init(TwoWire * wire)
        {
            wire->begin();
            wire->setClock(400000);
            delay(100);

            _vl53l1x.setBus(wire);

            if (!_vl53l1x.init()) {
                Debugger::error("ZRanger");
            }

            _vl53l1x.setDistanceMode(VL53L1X::Medium);
            _vl53l1x.setMeasurementTimingBudget(25000);

            _vl53l1x.startContinuous(50);
        }

        void step(EKF * ekf)
        {
            const float expCoeff =
                logf(EXP_STD_B / EXP_STD_A) / (EXP_POINT_B - EXP_POINT_A);

            float range = _vl53l1x.read();

            // check if range is feasible and push into the ekf the
            // sensor should not be able to measure >5 [m], and outliers
            // typically occur as >8 [m] measurements
            if (range < OUTLIER_LIMIT_MM) {

                float distance = range / 1000; // Scale from [mm] to [m]

                float stdDev = EXP_STD_A * (
                        1 + expf(expCoeff * (distance - EXP_POINT_A)));

                tofMeasurement_t tofData;
                tofData.distance = distance;
                tofData.stdDev = stdDev;

                ekf->enqueueRange(&tofData);
            }
        }

    private:

        static constexpr float TASK_FREQ = 40;

        static const uint16_t OUTLIER_LIMIT_MM = 5000;

        // Measurement noise model
        static constexpr float EXP_POINT_A = 2.5;
        static constexpr float EXP_STD_A = 0.0025; 
        static constexpr float EXP_POINT_B = 4.0;
        static constexpr float EXP_STD_B = 0.2;   

        VL53L1X _vl53l1x;
};
