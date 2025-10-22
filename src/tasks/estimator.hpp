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

#include <clock.hpp>
#include <ekf.hpp>
#include <rateSupervisor.hpp>
#include <task.hpp>
#include <tasks/debug.hpp>

class EstimatorTask {

    public:

        void begin(DebugTask * debugTask=nullptr)
        {
            _debugTask = debugTask;

            // Created in the 'empty' state, meaning the semaphore must first
            // be given, that is it will block in the task until released by
            // the stabilizer loop
            _runTaskSemaphore = xSemaphoreCreateBinary();

            _dataMutex = xSemaphoreCreateMutexStatic(&_dataMutexBuffer);

            _measurementsQueue = xQueueCreateStatic(
                    QUEUE_LENGTH, 
                    QUEUE_ITEM_SIZE,
                    measurementsQueueStorage,
                    &_measurementsQueueBuffer);

            _task.init(runEstimatorTask, "estimator", this, 4);

            _ekf.init(millis());
        }

        void getVehicleState(vehicleState_t * state)
        {
            // This function is called from the stabilizer loop. It is
            // important that this call returns as quickly as possible. The
            // dataMutex must only be
            // locked short periods by the task.
            xSemaphoreTake(_dataMutex, portMAX_DELAY);

            // Copy the latest state, calculated by the task
            memcpy(state, &_state, sizeof(vehicleState_t));
            xSemaphoreGive(_dataMutex);

            xSemaphoreGive(_runTaskSemaphore);
        }

        void setFlyingStatus(const bool isFlying)
        {
            _isFlying = isFlying;
        }

        void enqueueGyro(const Axis3f * gyro)
        {
            EKF::measurement_t m = {};
            m.type = EKF::MeasurementTypeGyroscope;
            m.data.gyroscope.gyro = *gyro;
            enqueue(&m);

            // Get state vector angular velocities directly from gyro
            _state.dphi   =  gyro->x;     
            _state.dtheta =  gyro->y;
            _state.dpsi   = -gyro->z; // negate for nose-right positive
        }

        void enqueueAccel(const Axis3f * accel)
        {
            EKF::measurement_t m = {};
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

    private:

        static const uint32_t WARNING_HOLD_BACK_TIME_MS = 2000;

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
        static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

        static const size_t QUEUE_LENGTH = 20;
        static const auto QUEUE_ITEM_SIZE = sizeof(EKF::measurement_t);
        uint8_t measurementsQueueStorage[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
        StaticQueue_t _measurementsQueueBuffer;
        xQueueHandle _measurementsQueue;

        FreeRtosTask _task;

        bool _didResetEstimation;

        RateSupervisor _rateSupervisor;
        
        bool _isFlying;

        // Mutex to protect data that is shared between the task and
        // functions called by the stabilizer loop
        xSemaphoreHandle _dataMutex;
        StaticSemaphore_t _dataMutexBuffer;

        // Semaphore to signal that we got data from the stabilizer loop to
        // process
        xSemaphoreHandle _runTaskSemaphore;

        uint32_t _warningBlockTimeMs;

        DebugTask * _debugTask;

        EKF _ekf;

        // Data used to enable the task and stabilizer loop to run with minimal
        // locking The estimator state produced by the task, copied to the
        // stabilizer when needed.
        vehicleState_t _state;

        uint32_t step(const uint32_t nowMs, uint32_t nextPredictionMs) 
        {
            xSemaphoreTake(_runTaskSemaphore, portMAX_DELAY);

            if (_didResetEstimation) {
                _ekf.init(nowMs);
               _didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (nowMs >= nextPredictionMs) {

                _ekf.predict(nowMs, _isFlying); 

                nextPredictionMs = nowMs + PREDICTION_UPDATE_INTERVAL_MS;
            }

            // Add process noise every loop, rather than every prediction
            _ekf.addProcessNoise(nowMs);

            // Sensor measurements can come in sporadically and faster
            // than the stabilizer loop frequency, we therefore consume all
            // measurements since the last loop, rather than accumulating

            // Pull the latest sensors values of interest; discard the rest

            EKF::measurement_t m = {};

            while (pdTRUE == xQueueReceive(_measurementsQueue, &m, 0)) {

                _ekf.update(m, nowMs);
            }

            _ekf.finalize();

            if (!_ekf.isStateWithinBounds()) {

                _didResetEstimation = true;

                if (nowMs > _warningBlockTimeMs) {
                    _warningBlockTimeMs = nowMs + WARNING_HOLD_BACK_TIME_MS;
                }
            }

            xSemaphoreTake(_dataMutex, portMAX_DELAY);
            _ekf.getVehicleState(_state);
            xSemaphoreGive(_dataMutex);

            return nextPredictionMs;
        }

        static void runEstimatorTask(void * obj) 
        {
            ((EstimatorTask *)obj)->run();
        }

        void run(void)
        {
            auto nextPredictionMs = millis();

            _rateSupervisor.init(
                    nextPredictionMs, 
                    1000, 
                    PREDICT_RATE - 1, 
                    PREDICT_RATE + 1, 
                    1); 

            while (true) {

                // would be nice if this had a precision higher than 1ms...
                nextPredictionMs = step(millis(), nextPredictionMs);
            }
        }

        void enqueue(const EKF::measurement_t * measurement) 
        {
            if (!_measurementsQueue) {
                return;
            }

            xQueueSend(_measurementsQueue, measurement, 0);
        }
};
