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

#include <datatypes.h>
#include <kalman.hpp>
#include <ladybug/task.hpp>

class EstimatorTask : public LadybugTask {

    public:

        EstimatorTask(void)
            : LadybugTask(ESTIMATOR, Clock::RATE_100_HZ)
        {
            _predictionUpdateIntervalMsec = 1000 / PREDICT_RATE;
        }

        void begin(void)
        {
            _kalmanFilter.setDefaultParams();

            _kalmanFilter.init(millis());
        }

        void run(const quaternion_t quat, vehicleState_t & state)
        {
            auto msec = millis();

            if (msec >= _nextPredictionMs) {

                _kalmanFilter.predict(msec, true); 

                _nextPredictionMs  = msec + _predictionUpdateIntervalMsec;
            }

            _kalmanFilter.addProcessNoise(msec);

            _kalmanFilter.updateWithQuaternion(quat);

            _kalmanFilter.finalize();

            _kalmanFilter.getVehicleState(state);

            // Rotate 180 degrees
            state.phi = -state.phi;
            state.theta = -state.theta;
        }

    private:

        static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 

        static float rad2deg(float rad)
        {
            return 180 * rad / M_PI;
        }

        KalmanFilter _kalmanFilter;

        uint32_t _predictionUpdateIntervalMsec;

        uint32_t _nextPredictionMs;

}; // class EstimatorTask
