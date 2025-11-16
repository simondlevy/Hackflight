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

#include <Arduino.h>

#include <debugger.hpp>
#include <ekf.hpp>

class Estimator {

    public:

        void begin()
        {
            _ekf.init(millis());
        }

        void enqueueImu(const axis3_t * gyro, const axis3_t * accel)
        {
            EKF::measurement_t m = {};
            m.type = EKF::MeasurementTypeGyroscope;
            m.data.gyroscope.gyro = *gyro;
            enqueue(&m);

            // Get state vector angular velocities directly from gyro
            _dangle.x = gyro->x;     
            _dangle.y = gyro->y;
            _dangle.z = -gyro->z; // negate for nose-right positive
            _state.dphi   =  gyro->x;     
            _state.dtheta =  gyro->y;
            _state.dpsi   = -gyro->z; // negate for nose-right positive

            m = {};
            m.type = EKF::MeasurementTypeAcceleration;
            m.data.acceleration.acc = *accel;
            enqueue(&m);
        }

        void enqueueFlow(const flowMeasurement_t * flow)
        {
            EKF::measurement_t m = {};
            m.type = EKF::MeasurementTypeFlow;
            m.data.flow = *flow;
            enqueue(&m);
        }

        void enqueueRange(const tofMeasurement_t * tof)
        {
            EKF::measurement_t m = {};
            m.type = EKF::MeasurementTypeTOF;
            m.data.tof = *tof;
            enqueue(&m);
        }

        void step(
                const uint32_t nowMs,
                const bool isFlying,
                uint32_t nextPredictionMs,
                vehicleState_t * state) 
        {
            if (_didResetEstimation) {
                _ekf.init(nowMs);
               _didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (nowMs >= nextPredictionMs) {
                _ekf.predict(nowMs, isFlying); 
                nextPredictionMs = nowMs + (1000 / EKF_PREDICTION_FREQ);
            }

            // Add process noise every loop, rather than every prediction
            _ekf.addProcessNoise(nowMs);

            // Pull the latest sensors values of interest; discard the rest
            for (uint32_t k=0; k<_queueLength; ++k) {
                _ekf.update(_measurementsQueue[k], nowMs);
            }
            _queueLength = 0;

            _ekf.finalize();

            if (!_ekf.isStateWithinBounds()) {
                _didResetEstimation = true;
            }

            _ekf.getVehicleState(_state);
            
            memcpy(state, &_state, sizeof(vehicleState_t));

            state->dphi   = _dangle.x;
            state->dtheta = _dangle.y;
            state->dpsi   = _dangle.z;
        }

    private:

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t EKF_PREDICTION_FREQ = 100;

        static const size_t QUEUE_MAX_LENGTH = 20;
        static const auto QUEUE_ITEM_SIZE = sizeof(EKF::measurement_t);

        EKF::measurement_t _measurementsQueue[QUEUE_MAX_LENGTH];
        uint32_t _queueLength;

        bool _didResetEstimation;

        EKF _ekf;

        vehicleState_t _state;

        axis3_t _dangle;

        void enqueue(const EKF::measurement_t * measurement) 
        {
            memcpy(&_measurementsQueue[_queueLength], measurement,
                    sizeof(EKF::measurement_t));
            _queueLength = (_queueLength + 1) % QUEUE_MAX_LENGTH;
        }
};
