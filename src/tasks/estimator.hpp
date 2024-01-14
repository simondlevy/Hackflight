/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2024 Simon D. Levy
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

#include <semphr.h>

#include <crossplatform.h>
#include <kalman.hpp>
#include <rateSupervisor.hpp>
#include <safety.hpp>
#include <task.hpp>

class EstimatorTask : public FreeRTOSTask {

    public:

        // Shared with params
        bool didResetEstimation;

        void begin(Safety * safety)
        {
            _safety = safety;

            // Created in the 'empty' state, meaning the semaphore must first be given,
            // that is it will block in the task until released by the stabilizer loop
            _runTaskSemaphore = xSemaphoreCreateBinary();

            _dataMutex = xSemaphoreCreateMutexStatic(&_dataMutexBuffer);

            _kalmanFilter.setDefaultParams();

            _measurementsQueue = xQueueCreateStatic(
                    QUEUE_LENGTH, 
                    QUEUE_ITEM_SIZE,
                    measurementsQueueStorage,
                    &measurementsQueueBuffer);

            FreeRTOSTask::begin(runEstimatorTask, "estimator", this, 4);

            consolePrintf("ESTIMATOR: estimatorTaskStart\n");

            _kalmanFilter.init(msec());
            consolePrintf("ESTIMATOR: Using %s estimator\n", _kalmanFilter.getName());              }

        bool didInit(void)
        {
            return _kalmanFilter.didInit();
        }

        void getVehicleState(vehicleState_t * state)
        {
            // This function is called from the stabilizer loop. It is important that
            // this call returns as quickly as possible. The dataMutex must only be
            // locked short periods by the task.
            xSemaphoreTake(_dataMutex, portMAX_DELAY);

            // Copy the latest state, calculated by the task
            memcpy(state, &_state, sizeof(vehicleState_t));
            xSemaphoreGive(_dataMutex);

            xSemaphoreGive(_runTaskSemaphore);
        }

        void enqueueGyro(const Axis3f * gyro, const bool isInInterrupt)
        {
            KalmanFilter::measurement_t m = {};
            m.type = KalmanFilter::MeasurementTypeGyroscope;
            m.data.gyroscope.gyro = *gyro;
            enqueue(&m, isInInterrupt);
        }

        void enqueueAccel(const Axis3f * accel, const bool isInInterrupt)
        {
            KalmanFilter::measurement_t m = {};
            m.type = KalmanFilter::MeasurementTypeAcceleration;
            m.data.acceleration.acc = *accel;
            enqueue(&m, isInInterrupt);
        }

        void enqueueBaro(const baro_t * baro, const bool isInInterrupt)
        {
            KalmanFilter::measurement_t m = {};
            m.type = KalmanFilter::MeasurementTypeBarometer;
            m.data.barometer.baro = *baro;
            enqueue(&m, isInInterrupt);
        }

        void enqueueFlow(const flowMeasurement_t * flow, const bool isInInterrupt)
        {
            KalmanFilter::measurement_t m = {};
            m.type = KalmanFilter::MeasurementTypeFlow;
            m.data.flow = *flow;
            enqueue(&m, isInInterrupt);
        }

        void enqueueRange(const tofMeasurement_t * tof, const bool isInInterrupt)
        {
            KalmanFilter::measurement_t m = {};
            m.type = KalmanFilter::MeasurementTypeTOF;
            m.data.tof = *tof;
            enqueue(&m, isInInterrupt);
        }

        // For VisualizerTask
        void getEulerAngles(int16_t angles[3])
        {
            static int16_t phi;
            static int8_t dir;

            dir = 
                dir == 0 ? +1 : 
                phi == +450 ? -1 :
                phi == -450 ? +1 :
                dir;

            phi += dir;

            angles[0] = phi;
            angles[1] = 0;
            angles[2] = 0;
        }

    private:

        static const uint32_t WARNING_HOLD_BACK_TIME_MS = 2000;

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
        static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

        static const size_t QUEUE_LENGTH = 20;
        static const auto QUEUE_ITEM_SIZE = sizeof(KalmanFilter::measurement_t);
        uint8_t measurementsQueueStorage[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
        StaticQueue_t measurementsQueueBuffer;
        xQueueHandle _measurementsQueue;

        RateSupervisor _rateSupervisor;

        // Mutex to protect data that is shared between the task and
        // functions called by the stabilizer loop
        SemaphoreHandle_t _dataMutex;
        StaticSemaphore_t _dataMutexBuffer;

        // Semaphore to signal that we got data from the stabilizer loop to process
        SemaphoreHandle_t _runTaskSemaphore;

        uint32_t _warningBlockTimeMs;

        Safety * _safety;

        KalmanFilter _kalmanFilter;

        // Data used to enable the task and stabilizer loop to run with minimal locking
        // The estimator state produced by the task, copied to the stabilizer when needed.
        vehicleState_t _state;

        static uint32_t msec(void)
        {
            return T2M(xTaskGetTickCount());
        }

        uint32_t step(const uint32_t nowMs, uint32_t nextPredictionMs) 
        {
            xSemaphoreTake(_runTaskSemaphore, portMAX_DELAY);

            if (didResetEstimation) {
                _kalmanFilter.init(nowMs);
                didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (nowMs >= nextPredictionMs) {

                _kalmanFilter.predict(nowMs, _safety->isFlying()); 

                nextPredictionMs = nowMs + PREDICTION_UPDATE_INTERVAL_MS;

                if (!_rateSupervisor.validate(nowMs)) {
                    consolePrintf(
                            "ESTIMATOR: WARNING: Kalman prediction rate off (%lu)\n", 
                            _rateSupervisor.getLatestCount());
                }
            }

            // Add process noise every loop, rather than every prediction
            _kalmanFilter.addProcessNoise(nowMs);

            // Sensor measurements can come in sporadically and faster
            // than the stabilizer loop frequency, we therefore consume all
            // measurements since the last loop, rather than accumulating

            // Pull the latest sensors values of interest; discard the rest

            KalmanFilter::measurement_t m = {};

            while (pdTRUE == xQueueReceive(_measurementsQueue, &m, 0)) {

                _kalmanFilter.update(m, nowMs);
            }

            _kalmanFilter.finalize();

            if (!_kalmanFilter.isStateWithinBounds()) {

                didResetEstimation = true;

                if (nowMs > _warningBlockTimeMs) {
                    _warningBlockTimeMs = nowMs + WARNING_HOLD_BACK_TIME_MS;
                    consolePrintf("ESTIMATOR: State out of bounds, resetting\n");
                }
            }

            xSemaphoreTake(_dataMutex, portMAX_DELAY);
            _kalmanFilter.getVehicleState(_state);
            xSemaphoreGive(_dataMutex);

            return nextPredictionMs;
        }

        static void runEstimatorTask(void * obj) 
        {
            ((EstimatorTask *)obj)->run();
        }

        void run(void)
        {
            consolePrintf("ESTIMATOR: running\n");

            systemWaitStart();

            auto nextPredictionMs = msec();

            _rateSupervisor.init(
                    nextPredictionMs, 
                    1000, 
                    PREDICT_RATE - 1, 
                    PREDICT_RATE + 1, 
                    1); 

            while (true) {

                // would be nice if this had a precision higher than 1ms...
                nextPredictionMs = step(msec(), nextPredictionMs);
            }
        }

        void enqueue(
                const KalmanFilter::measurement_t * measurement, 
                const bool isInInterrupt)
        {
            if (!_measurementsQueue) {
                return;
            }

            if (isInInterrupt) {
                auto xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(
                        _measurementsQueue, measurement, &xHigherPriorityTaskWoken);
                if (xHigherPriorityTaskWoken == pdTRUE) {
                    portYIELD();
                }
            } else {
                xQueueSend(_measurementsQueue, measurement, 0);
            }
        }
};
