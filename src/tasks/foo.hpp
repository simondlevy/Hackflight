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

#include <tasks/debug.hpp>
#include <tasks/estimator.hpp>

#include <datatypes.h>
#include <lpf.hpp>
#include <m_pi.h>
#include <num.hpp>
#include <time.h>

class FooTask {

    private:

        static constexpr float CALIBRATION_PITCH = 0;
        static constexpr float CALIBRATION_ROLL = 0;

    public:

        void begin(EstimatorTask * estimatorTask,
                DebugTask * debugTask)
        {
            if (_task.didInit()) {
                return;
            }

            // Wait for sensors to startup
            vTaskDelay(STARTUP_TIME_MS);

            _estimatorTask = estimatorTask;

            _debugTask = debugTask;

            _task.init(runFooTask, "foo", this, 3);
        }

    private:

        static constexpr float RAW_GYRO_VARIANCE_BASE = 100;

        static const uint32_t STARTUP_TIME_MS = 1000;
        static const uint32_t ACC_SCALE_SAMPLES = 200;

        // IMU alignment on the airframe 
        static constexpr float ALIGN_PHI   = 0;
        static constexpr float ALIGN_THETA = 0;
        static constexpr float ALIGN_PSI   = 0;

        // Number of samples used in variance calculation. Changing this
        // effects the threshold
        static const uint16_t NBR_OF_BIAS_SAMPLES = 512;

        static constexpr float GYRO_LPF_CUTOFF_FREQ  = 80;
        static constexpr float ACCEL_LPF_CUTOFF_FREQ = 30;

        static const uint32_t GYRO_MIN_BIAS_TIMEOUT_MS = 1000;

        typedef union {
            struct {
                int16_t x;
                int16_t y;
                int16_t z;
            };
            int16_t axis[3];
        } Axis3i16;

        typedef struct {

            Axis3f     bias;
            Axis3f     variance;
            Axis3f     mean;
            bool       isBiasValueFound;
            bool       isBufferFilled;
            Axis3i16*  bufHead;
            Axis3i16   buffer[NBR_OF_BIAS_SAMPLES];

        } bias_t;

        static void calculateVarianceAndMean(
                bias_t* bias, Axis3f* varOut, Axis3f* meanOut)
        {
            int64_t sum[3] = {};
            int64_t sumSq[3] = {};

            for (uint16_t i=0; i<NBR_OF_BIAS_SAMPLES; i++) {

                sum[0] += bias->buffer[i].x;
                sum[1] += bias->buffer[i].y;
                sum[2] += bias->buffer[i].z;
                sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
                sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
                sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
            }

            meanOut->x = (float) sum[0] / NBR_OF_BIAS_SAMPLES;
            meanOut->y = (float) sum[1] / NBR_OF_BIAS_SAMPLES;
            meanOut->z = (float) sum[2] / NBR_OF_BIAS_SAMPLES;

            varOut->x =
                sumSq[0] / NBR_OF_BIAS_SAMPLES - meanOut->x * meanOut->x;
            varOut->y =
                sumSq[1] / NBR_OF_BIAS_SAMPLES - meanOut->y * meanOut->y;
            varOut->z =
                sumSq[2] / NBR_OF_BIAS_SAMPLES - meanOut->z * meanOut->z;
        }

        static const uint8_t QUEUE_LENGTH = 1;

        static const auto IMU_ITEM_SIZE = sizeof(Axis3f);

        static const auto IMU_QUEUE_LENGTH = QUEUE_LENGTH * IMU_ITEM_SIZE;

        uint8_t _accelQueueStorage[IMU_QUEUE_LENGTH];
        StaticQueue_t _accelQueueBuffer;
        QueueHandle_t _accelQueue;

        uint8_t _gyroQueueStorage[IMU_QUEUE_LENGTH];
        StaticQueue_t _gyroQueueBuffer;
        QueueHandle_t _gyroQueue;

        bias_t _gyroBiasRunning;

        FreeRtosTask _task;

        EstimatorTask * _estimatorTask;

        DebugTask * _debugTask;

        static void runFooTask(void *obj)
        {
            ((FooTask *)obj)->run();
        }

        void run(void)
        {
            while (true) {

                static uint32_t count;
                DebugTask::setMessage(_debugTask, "FooTask: %d", +count++);
                vTaskDelay(1);

            }
        }
};
