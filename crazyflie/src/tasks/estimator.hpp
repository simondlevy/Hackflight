#pragma once

#include <free_rtos.h>
#include <semphr.h>
#include <task.h>

#include <estimator.hpp>
#include <rateSupervisor.hpp>

#include <crossplatform.h>
#include <safety.hpp>
#include <system.h>

class EstimatorTask {

    public:

        // Shared with logger
        Estimator::kalmanCoreData_t kalmanData;        
        float predictedNX;
        float predictedNY;
        float measuredNX;
        float measuredNY;

        // Shared with params
        bool didResetEstimation;

        void init(Safety * safety)
        {
            _safety = safety;

            // Created in the 'empty' state, meaning the semaphore must first be given,
            // that is it will block in the task until released by the stabilizer loop
            _runTaskSemaphore = xSemaphoreCreateBinary();

            _dataMutex = xSemaphoreCreateMutexStatic(&_dataMutexBuffer);

            _estimator.setDefaultParams();

            _measurementsQueue = xQueueCreateStatic(
                    QUEUE_LENGTH, 
                    QUEUE_ITEM_SIZE,
                    measurementsQueueStorage,
                    &measurementsQueueBuffer);

            xTaskCreateStatic(
                    estimatorTask, 
                    "ESTIMATOR",
                    TASK_STACK_DEPTH,
                    this, 
                    2, 
                    taskStackBuffer,
                    &taskTaskBuffer);

            consolePrintf("ESTIMATOR: estimatorTaskStart\n");

            _estimator.init(msec());
            consolePrintf("ESTIMATOR: Using %s estimator\n", _estimator.getName());        }

        bool didInit(void)
        {
            return _estimator.didInit();
        }

        void getState(state_t * state)
        {
            // This function is called from the stabilizer loop. It is important that
            // this call returns as quickly as possible. The dataMutex must only be
            // locked short periods by the task.
            xSemaphoreTake(_dataMutex, portMAX_DELAY);

            // Copy the latest state, calculated by the task
            memcpy(state, &_state, sizeof(state_t));
            xSemaphoreGive(_dataMutex);

            xSemaphoreGive(_runTaskSemaphore);

            memcpy(&kalmanData, &_estimator._kalmanData, 
                    sizeof(Estimator::kalmanCoreData_t));

            predictedNX = _estimator._predictedNX;
            predictedNY = _estimator._predictedNY;

            measuredNX = _estimator._measuredNX;
            measuredNY = _estimator._measuredNY;
        }

        void enqueueGyro(const Axis3f * gyro, const bool isInInterrupt)
        {
            Estimator::measurement_t m = {};
            m.type = Estimator::MeasurementTypeGyroscope;
            m.data.gyroscope.gyro = *gyro;
            enqueue(&m, isInInterrupt);
        }

        void enqueueAccel(const Axis3f * accel, const bool isInInterrupt)
        {
            Estimator::measurement_t m = {};
            m.type = Estimator::MeasurementTypeAcceleration;
            m.data.acceleration.acc = *accel;
            enqueue(&m, isInInterrupt);
        }

        void enqueueBaro(const baro_t * baro, const bool isInInterrupt)
        {
            Estimator::measurement_t m = {};
            m.type = Estimator::MeasurementTypeBarometer;
            m.data.barometer.baro = *baro;
            enqueue(&m, isInInterrupt);
        }

        void enqueueFlow(const flowMeasurement_t * flow, const bool isInInterrupt)
        {
            Estimator::measurement_t m = {};
            m.type = Estimator::MeasurementTypeFlow;
            m.data.flow = *flow;
            enqueue(&m, isInInterrupt);
        }

        void enqueueRange(const tofMeasurement_t * tof, const bool isInInterrupt)
        {
            Estimator::measurement_t m = {};
            m.type = Estimator::MeasurementTypeTOF;
            m.data.tof = *tof;
            enqueue(&m, isInInterrupt);
        }

    private:

        static const uint32_t WARNING_HOLD_BACK_TIME_MS = 2000;

        // this is slower than the IMU update rate of 1000Hz
        static const uint32_t PREDICT_RATE = Clock::RATE_100_HZ; 
        static const uint32_t PREDICTION_UPDATE_INTERVAL_MS = 1000 / PREDICT_RATE;

        static const auto TASK_STACK_DEPTH = 3 * configMINIMAL_STACK_SIZE;
        StackType_t  taskStackBuffer[TASK_STACK_DEPTH]; 
        StaticTask_t taskTaskBuffer;

        static const size_t QUEUE_LENGTH = 20;
        static const auto QUEUE_ITEM_SIZE = sizeof(Estimator::measurement_t);
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

        Estimator _estimator;

        // Data used to enable the task and stabilizer loop to run with minimal locking
        // The estimator state produced by the task, copied to the stabilizer when needed.
        state_t _state;

        static uint32_t msec(void)
        {
            return T2M(xTaskGetTickCount());
        }

        uint32_t step(const uint32_t nowMs, uint32_t nextPredictionMs) 
        {
            xSemaphoreTake(_runTaskSemaphore, portMAX_DELAY);

            if (didResetEstimation) {
                _estimator.init(nowMs);
                didResetEstimation = false;
            }

            // Run the system dynamics to predict the state forward.
            if (nowMs >= nextPredictionMs) {

                _estimator.predict(nowMs, _safety->isFlying()); 

                nextPredictionMs = nowMs + PREDICTION_UPDATE_INTERVAL_MS;

                if (!_rateSupervisor.validate(nowMs)) {
                    consolePrintf(
                            "ESTIMATOR: WARNING: Kalman prediction rate off (%lu)\n", 
                            _rateSupervisor.getLatestCount());
                }
            }

            // Add process noise every loop, rather than every prediction
            _estimator.addProcessNoise(nowMs);

            // Sensor measurements can come in sporadically and faster
            // than the stabilizer loop frequency, we therefore consume all
            // measurements since the last loop, rather than accumulating

            // Pull the latest sensors values of interest; discard the rest

            Estimator::measurement_t m = {};

            while (pdTRUE == xQueueReceive(_measurementsQueue, &m, 0)) {

                _estimator.update(m, nowMs);
            }

            _estimator.finalize();

            if (!_estimator.isStateWithinBounds()) {

                didResetEstimation = true;

                if (nowMs > _warningBlockTimeMs) {
                    _warningBlockTimeMs = nowMs + WARNING_HOLD_BACK_TIME_MS;
                    consolePrintf("ESTIMATOR: State out of bounds, resetting\n");
                }
            }

            xSemaphoreTake(_dataMutex, portMAX_DELAY);
            _estimator.getState(_state);
            xSemaphoreGive(_dataMutex);

            return nextPredictionMs;
        }

        static void estimatorTask(void * parameters) 
        {
            systemWaitStart();

            auto task = (EstimatorTask *)parameters;

            auto nextPredictionMs = msec();

            task->_rateSupervisor.init(
                    nextPredictionMs, 
                    1000, 
                    PREDICT_RATE - 1, 
                    PREDICT_RATE + 1, 
                    1); 

            while (true) {

                // would be nice if this had a precision higher than 1ms...
                nextPredictionMs = task->step(msec(), nextPredictionMs);
            }
        }

        void enqueue(
                const Estimator::measurement_t * measurement, 
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
