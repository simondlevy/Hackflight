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

#include <math.h>

#include <task.hpp>
#include <tasks/estimator.hpp>

#include <time.h>
#include <lpf.hpp>
#include <num.hpp>
#include <m_pi.h>
#include <datatypes.h>

class ImuTask : public FreeRTOSTask {

    public:

        // Called from main program
        void begin(
                EstimatorTask * estimatorTask, 
                const float calibRoll,
                const float calibPitch)
        {
            if (didInit) {
                return;
            }

            _estimatorTask = estimatorTask;

            // Wait for sensors to startup
            vTaskDelay(M2T(STARTUP_TIME_MS));

            gyroBiasRunning.isBufferFilled = false;
            gyroBiasRunning.bufHead = gyroBiasRunning.buffer;

            // Create a semaphore to protect data-ready interrupts from IMU
            interruptCallbackSemaphore = xSemaphoreCreateBinaryStatic(
                    &interruptCallbackSemaphoreBuffer);

            // Create a semaphore to protect waitDataReady() calls from core task
            coreTaskSemaphore = xSemaphoreCreateBinaryStatic(&coreTaskSemaphoreBuffer);

            // Start our IMU (e.g. BMI088)
            deviceInit();

            // Calibrate
            for (uint8_t i = 0; i < 3; i++) {
                _gyroLpf[i].init(1000, GYRO_LPF_CUTOFF_FREQ);
                _accLpf[i].init(1000, ACCEL_LPF_CUTOFF_FREQ);
            }
            _cosPitch = cosf(calibPitch * (float) M_PI / 180);
            _sinPitch = sinf(calibPitch * (float) M_PI / 180);
            _cosRoll = cosf(calibRoll * (float) M_PI / 180);
            _sinRoll = sinf(calibRoll * (float) M_PI / 180);

            accelQueue = makeImuQueue(accelQueueStorage, &accelQueueBuffer);

            gyroQueue = makeImuQueue(gyroQueueStorage, &gyroQueueBuffer);

            FreeRTOSTask::begin(runImuTask, "imu", this, 3);

            didInit = true;
        }

        // Called by platform-specific IMU interrupt
        void dataAvailableCallback(void)
        {
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            interruptTimestamp = micros();
            xSemaphoreGiveFromISR(interruptCallbackSemaphore, &xHigherPriorityTaskWoken);

            if (xHigherPriorityTaskWoken) {
                portYIELD();
            }
        }

        // Called by core task
        bool test(void)
        {
            bool testStatus = true;

            if (!didInit) {
                testStatus = false;
            }

            return testStatus;
        }

        // Called by core task
        bool areCalibrated() {
            return gyroBiasFound;
        }

        // Called by core task
        void waitDataReady(void) {
            xSemaphoreTake(coreTaskSemaphore, portMAX_DELAY);
        }

    private:

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

            varOut->x = sumSq[0] / NBR_OF_BIAS_SAMPLES - meanOut->x * meanOut->x;
            varOut->y = sumSq[1] / NBR_OF_BIAS_SAMPLES - meanOut->y * meanOut->y;
            varOut->z = sumSq[2] / NBR_OF_BIAS_SAMPLES - meanOut->z * meanOut->z;
        }

        static const uint8_t QUEUE_LENGTH = 1;

        static const auto IMU_ITEM_SIZE = sizeof(Axis3f);

        static const auto IMU_QUEUE_LENGTH = QUEUE_LENGTH * IMU_ITEM_SIZE;

        uint8_t accelQueueStorage[IMU_QUEUE_LENGTH];
        StaticQueue_t accelQueueBuffer;
        xQueueHandle accelQueue;

        uint8_t gyroQueueStorage[IMU_QUEUE_LENGTH];
        StaticQueue_t gyroQueueBuffer;
        xQueueHandle gyroQueue;

        bias_t gyroBiasRunning;

        EstimatorTask * _estimatorTask;

        /**
         * Checks if the variances is below the predefined thresholds.
         * The bias value should have been added before calling this.
         * @param bias  The bias object
         */
        bool findBiasValue(const uint32_t ticks)
        {
            static int32_t varianceSampleTime;
            bool foundBias = false;

            if (gyroBiasRunning.isBufferFilled)
            {
                calculateVarianceAndMean( &gyroBiasRunning,
                        &gyroBiasRunning.variance, &gyroBiasRunning.mean);

                if (gyroBiasRunning.variance.x < rawGyroVarianceBase() &&
                        gyroBiasRunning.variance.y < rawGyroVarianceBase() &&
                        gyroBiasRunning.variance.z < rawGyroVarianceBase() &&
                        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < ticks))
                {
                    varianceSampleTime = ticks;
                    gyroBiasRunning.bias.x = gyroBiasRunning.mean.x;
                    gyroBiasRunning.bias.y = gyroBiasRunning.mean.y;
                    gyroBiasRunning.bias.z = gyroBiasRunning.mean.z;
                    foundBias = true;
                    gyroBiasRunning.isBiasValueFound = true;
                }
            }

            return foundBias;
        }

        static void applyLpf(Lpf lpf[3], Axis3f* in)
        {
            for (uint8_t i = 0; i < 3; i++) {
                in->axis[i] = lpf[i].apply(in->axis[i]);
            }
        }

        // Low Pass filtering
        Lpf _accLpf[3];
        Lpf _gyroLpf[3];

        // Pre-calculated values for accelerometer alignment
        float _cosPitch;
        float _sinPitch;
        float _cosRoll;
        float _sinRoll;

        static xQueueHandle makeImuQueue(
                uint8_t storage[], StaticQueue_t * buffer)
        {
            return makeQueue(IMU_ITEM_SIZE, storage, buffer);

        }

        static xQueueHandle makeQueue(
                const uint8_t itemSize, uint8_t storage[], StaticQueue_t * buffer)
        {
            return xQueueCreateStatic(QUEUE_LENGTH, itemSize, storage, buffer);

        }

        static void alignToAirframe(Axis3f* in, Axis3f* out)
        {
            static float R[3][3];

            // IMU alignment
            static float sphi, cphi, stheta, ctheta, spsi, cpsi;

            sphi   = sinf(ALIGN_PHI * (float) M_PI / 180);
            cphi   = cosf(ALIGN_PHI * (float) M_PI / 180);
            stheta = sinf(ALIGN_THETA * (float) M_PI / 180);
            ctheta = cosf(ALIGN_THETA * (float) M_PI / 180);
            spsi   = sinf(ALIGN_PSI * (float) M_PI / 180);
            cpsi   = cosf(ALIGN_PSI * (float) M_PI / 180);

            R[0][0] = ctheta * cpsi;
            R[0][1] = ctheta * spsi;
            R[0][2] = -stheta;
            R[1][0] = sphi * stheta * cpsi - cphi * spsi;
            R[1][1] = sphi * stheta * spsi + cphi * cpsi;
            R[1][2] = sphi * ctheta;
            R[2][0] = cphi * stheta * cpsi + sphi * spsi;
            R[2][1] = cphi * stheta * spsi - sphi * cpsi;
            R[2][2] = cphi * ctheta;

            out->x = in->x*R[0][0] + in->y*R[0][1] + in->z*R[0][2];
            out->y = in->x*R[1][0] + in->y*R[1][1] + in->z*R[1][2];
            out->z = in->x*R[2][0] + in->y*R[2][1] + in->z*R[2][2];
        }

        bool gyroBiasFound;
        sensorData_t data;
        Axis3f gyroBias;
        volatile uint64_t interruptTimestamp;

        xSemaphoreHandle interruptCallbackSemaphore;
        StaticSemaphore_t interruptCallbackSemaphoreBuffer;

        xSemaphoreHandle coreTaskSemaphore;
        StaticSemaphore_t coreTaskSemaphoreBuffer;

        /**
         * Compensate for a miss-aligned accelerometer. It uses the trim
         * data gathered from the UI and written in the config-block to
         * rotate the accelerometer to be aligned with gravity.
         */
        void accAlignToGravity(Axis3f* in, Axis3f* out)
        {

            // Rotate around x-axis
            Axis3f rx = {};
            rx.x = in->x;
            rx.y = in->y * _cosRoll - in->z * _sinRoll;
            rx.z = in->y * _sinRoll + in->z * _cosRoll;

            // Rotate around y-axis
            Axis3f ry = {};
            ry.x = rx.x * _cosPitch - rx.z * _sinPitch;
            ry.y = rx.y;
            ry.z = -rx.x * _sinPitch + rx.z * _cosPitch;

            out->x = ry.x;
            out->y = ry.y;
            out->z = ry.z;
        }

        /**
         * Calculates the bias first when the gyro variance is below threshold.
         * Requires a buffer but calibrates platform first when it is stable.
         */
        bool processGyroBias(const uint32_t tickCount,
                const Axis3i16 gyroRaw, Axis3f *gyroBiasOut)
        {
            gyroBiasRunning.bufHead->x = gyroRaw.x;
            gyroBiasRunning.bufHead->y = gyroRaw.y;
            gyroBiasRunning.bufHead->z = gyroRaw.z;
            gyroBiasRunning.bufHead++;

            if (gyroBiasRunning.bufHead >= 
                    &gyroBiasRunning.buffer[NBR_OF_BIAS_SAMPLES]) {

                gyroBiasRunning.bufHead = gyroBiasRunning.buffer;
                gyroBiasRunning.isBufferFilled = true;
            }

            if (!gyroBiasRunning.isBiasValueFound) {
                findBiasValue(tickCount);
            }

            gyroBiasOut->x = gyroBiasRunning.bias.x;
            gyroBiasOut->y = gyroBiasRunning.bias.y;
            gyroBiasOut->z = gyroBiasRunning.bias.z;

            return gyroBiasRunning.isBiasValueFound;
        }

        static void runImuTask(void *obj)
        {
            ((ImuTask *)obj)->run();
        }

        void run(void)
        {
            void systemWaitStart(void);
            systemWaitStart();

            while (true) {

                if (pdTRUE == xSemaphoreTake(
                            interruptCallbackSemaphore, portMAX_DELAY)) {

                    data.interruptTimestamp = interruptTimestamp;

                    // Get raw data from IMU
                    Axis3i16 gyroRaw = {};
                    readGyroRaw(&gyroRaw);
                    Axis3i16 accelRaw = {};
                    readAccelRaw(&accelRaw);

                    // Convert accel to Gs
                    Axis3f accel = {};
                    accel.x = accelRaw2Gs(accelRaw.x);
                    accel.y = accelRaw2Gs(accelRaw.y);
                    accel.z = accelRaw2Gs(accelRaw.z);

                    // Calibrate gyro with raw values if necessary
                    gyroBiasFound = processGyroBias(xTaskGetTickCount(),
                            gyroRaw, &gyroBias);

                    // Subtract gyro bias
                    Axis3f gyroUnbiased = {};
                    gyroUnbiased.x = gyroRaw2Dps(gyroRaw.x - gyroBias.x);
                    gyroUnbiased.y = gyroRaw2Dps(gyroRaw.y - gyroBias.y);
                    gyroUnbiased.z = gyroRaw2Dps(gyroRaw.z - gyroBias.z);

                    // Rotate gyro to airframe
                    alignToAirframe(&gyroUnbiased, &data.gyro);

                    // LPF gyro
                    applyLpf(_gyroLpf, &data.gyro);

                    _estimatorTask->enqueueGyro(&data.gyro, xPortIsInsideInterrupt());

                    Axis3f accScaled = {};
                    alignToAirframe(&accel, &accScaled);

                    accAlignToGravity(&accScaled, &data.acc);

                    applyLpf(_accLpf, &data.acc);

                    _estimatorTask->enqueueAccel(&data.acc, xPortIsInsideInterrupt());
                }

                xQueueOverwrite(accelQueue, &data.acc);
                xQueueOverwrite(gyroQueue, &data.gyro);

                xSemaphoreGive(coreTaskSemaphore);
            }
        }

        // Hardware-dependent
        void deviceInit(void); 
        void readGyroRaw(Axis3i16 * dataOut);
        void readAccelRaw(Axis3i16 * dataOut);
        float gyroRaw2Dps(const int16_t raw);
        float accelRaw2Gs(const int16_t raw);
        const float rawGyroVarianceBase();
};
