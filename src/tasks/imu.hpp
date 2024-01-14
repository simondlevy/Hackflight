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

#include <console.h>
#include <crossplatform.h>
#include <lpf.hpp>
#include <num.hpp>
#include <physicalConstants.h>
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

            accScale = 1;
            accScaleFound = false;
            accScaleSumCount = 0;

            // Wait for sensors to startup
            vTaskDelay(M2T(STARTUP_TIME_MS));

            initGyroBias();

            deviceInit();

            calibrate(calibRoll, calibPitch);

            accelQueue = makeImuQueue(accelQueueStorage, &accelQueueBuffer);

            gyroQueue = makeImuQueue(gyroQueueStorage, &gyroQueueBuffer);

            magQueue = makeImuQueue(magQueueStorage, &magQueueBuffer);

            FreeRTOSTask::begin(runImuTask, "imu", this, 3);

            didInit = true;
        }

        // Called by platform-specific IMU interrupt
        void dataAvailableCallback(void)
        {
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            interruptTimestamp = micros();
            xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

            if (xHigherPriorityTaskWoken) {
                portYIELD();
            }
        }

        // Called by core task
        bool test(void)
        {
            bool testStatus = true;

            if (!didInit) {
                consolePrintf("IMU: Uninitialized\n");
                testStatus = false;
            }

            if (! gyroSelfTest()) {
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
            xSemaphoreTake(dataReady, portMAX_DELAY);
        }

        // Called by core task
        void acquire(sensorData_t *sensors)
        {
            xQueueReceive(gyroQueue, &sensors->gyro, 0);
            xQueueReceive(accelQueue, &sensors->acc, 0);
            xQueueReceive(magQueue, &sensors->mag, 0);

            sensors->interruptTimestamp = data.interruptTimestamp;
        }

    private:

        static const auto STACK_DEPTH = 3 * configMINIMAL_STACK_SIZE;

        static constexpr float DEG_PER_LSB = (2.0f *2000.0f) / 65536.0f;

        static const uint8_t ACCEL_CFG = 24;

        static constexpr float G_PER_LSB = (2.0f * (float)ACCEL_CFG) / 65536.0f;

        /**
         * IMU update frequency dictates the overall update frequency.
         */
        static constexpr float UPDATE_FREQ =  500;

        /**
         * Set ACC_WANTED_LPF1_CUTOFF_HZ to the wanted cut-off freq in Hz.
         * The highest cut-off freq that will have any affect is fs /(2*pi).
         * E.g. fs = 350 Hz -> highest cut-off = 350/(2*pi) = 55.7 Hz -> 55 Hz
         */
        static constexpr float ACC_WANTED_LPF_CUTOFF_HZ  = 4;

        /**
         * Attenuation should be between 1 to 256.
         *
         * f0 = fs / 2*pi*attenuation ->
         * attenuation = fs / 2*pi*f0
         */
        static constexpr float ACC_IIR_LPF_ATTENUATION = 
            (UPDATE_FREQ / (2 * 3.1415f * ACC_WANTED_LPF_CUTOFF_HZ));

        static const uint32_t ACC_IIR_LPF_ATT_FACTOR  =
            (uint32_t)(((1<<Lpf::IIR_SHIFT) / ACC_IIR_LPF_ATTENUATION) + 0.5f);

        static constexpr float UPDATE_DT =  1.0f / UPDATE_FREQ;

        static const uint32_t READ_RATE_HZ = 1000;
        static const uint32_t READ_BARO_HZ = 50;
        static const uint32_t READ_MAG_HZ = 20;
        static const uint32_t DELAY_MAG = READ_RATE_HZ/READ_MAG_HZ;
        static const uint32_t VARIANCE_MAN_TEST_TIMEOUT  = 1000; // Timeout in ms
        static constexpr float MAN_TEST_LEVEL_MAX = 5.0f;     // Max degrees off

        static const uint32_t STARTUP_TIME_MS = 1000;
        static const uint32_t ACC_SCALE_SAMPLES = 200;
        static const uint32_t DELAY_BARO = READ_RATE_HZ/READ_BARO_HZ;

        // IMU alignment on the airframe 
        static constexpr float ALIGN_PHI   = 0;
        static constexpr float ALIGN_THETA = 0;
        static constexpr float ALIGN_PSI   = 0;

        // Number of samples used in variance calculation. Changing this
        // effects the threshold
        static const uint16_t NBR_OF_BIAS_SAMPLES = 512;

        // Variance threshold to take zero bias for gyro
        static constexpr float GYRO_VARIANCE_BASE = 100;
        static constexpr float GYRO_VARIANCE_THRESHOLD_X = GYRO_VARIANCE_BASE;
        static constexpr float GYRO_VARIANCE_THRESHOLD_Y = GYRO_VARIANCE_BASE;
        static constexpr float GYRO_VARIANCE_THRESHOLD_Z = GYRO_VARIANCE_BASE;

        static constexpr float GYRO_LPF_CUTOFF_FREQ  = 80;
        static constexpr float ACCEL_LPF_CUTOFF_FREQ = 30;

        static const uint32_t GYRO_MIN_BIAS_TIMEOUT_MS = 1000;

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
            uint32_t i;
            int64_t sum[3] = {0};
            int64_t sumSq[3] = {0};

            for (i = 0; i < NBR_OF_BIAS_SAMPLES; i++)
            {
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

        uint8_t magQueueStorage[IMU_QUEUE_LENGTH];
        StaticQueue_t magQueueBuffer;
        xQueueHandle magQueue;

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

                if (gyroBiasRunning.variance.x < GYRO_VARIANCE_THRESHOLD_X &&
                        gyroBiasRunning.variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
                        gyroBiasRunning.variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
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

        /**
         * Adds a new value to the variance buffer and if it is full
         * replaces the oldest one. Thus a circular buffer.
         */
        void addBiasValue(int16_t x, int16_t y, int16_t z)
        {
            gyroBiasRunning.bufHead->x = x;
            gyroBiasRunning.bufHead->y = y;
            gyroBiasRunning.bufHead->z = z;
            gyroBiasRunning.bufHead++;

            if (gyroBiasRunning.bufHead >= 
                    &gyroBiasRunning.buffer[NBR_OF_BIAS_SAMPLES]) {

                gyroBiasRunning.bufHead = gyroBiasRunning.buffer;
                gyroBiasRunning.isBufferFilled = true;
            }
        }

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
        Axis3i16 gyroRaw;
        Axis3i16 accelRaw;
        Axis3f gyroBias;
        float accScale;
        bool accScaleFound;
        uint32_t accScaleSumCount;
        volatile uint64_t interruptTimestamp;

        xSemaphoreHandle sensorsDataReady;
        StaticSemaphore_t sensorsDataReadyBuffer;
        xSemaphoreHandle dataReady;
        StaticSemaphore_t dataReadyBuffer;

        void applyAccelLpf(Axis3f * accelVals)
        {
            applyLpf(_accLpf, accelVals);
        }

        void applyGyroLpf(Axis3f * gyroVals)
        {
            applyLpf(_gyroLpf, gyroVals);
        }

        void calibrate(const float calibRoll, const float calibPitch)
        {
            for (uint8_t i = 0; i < 3; i++) {
                _gyroLpf[i].init(1000, GYRO_LPF_CUTOFF_FREQ);
                _accLpf[i].init(1000, ACCEL_LPF_CUTOFF_FREQ);
            }

            _cosPitch = cosf(calibPitch * (float) M_PI / 180);
            _sinPitch = sinf(calibPitch * (float) M_PI / 180);
            _cosRoll = cosf(calibRoll * (float) M_PI / 180);
            _sinRoll = sinf(calibRoll * (float) M_PI / 180);
        }

        /**
         * Compensate for a miss-aligned accelerometer. It uses the trim
         * data gathered from the UI and written in the config-block to
         * rotate the accelerometer to be aligned with gravity.
         */
        void accAlignToGravity(Axis3f* in, Axis3f* out)
        {
            Axis3f rx;
            Axis3f ry;

            // Rotate around x-axis
            rx.x = in->x;
            rx.y = in->y * _cosRoll - in->z * _sinRoll;
            rx.z = in->y * _sinRoll + in->z * _cosRoll;

            // Rotate around y-axis
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
                int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
        {
            addBiasValue(gx, gy, gz);

            if (!gyroBiasRunning.isBiasValueFound) {
                findBiasValue(tickCount);
            }

            gyroBiasOut->x = gyroBiasRunning.bias.x;
            gyroBiasOut->y = gyroBiasRunning.bias.y;
            gyroBiasOut->z = gyroBiasRunning.bias.z;

            return gyroBiasRunning.isBiasValueFound;
        }

        void initGyroBias(void)
        {
            gyroBiasRunning.isBufferFilled = false;
            gyroBiasRunning.bufHead = gyroBiasRunning.buffer;
        }

        bool processAccelScale(int16_t ax, int16_t ay, int16_t az)
        {
            static float accScaleSum;

            if (!accScaleFound) {

                accScaleSum += sqrtf(
                        powf(ax * G_PER_LSB, 2) + 
                        powf(ay * G_PER_LSB, 2) + 
                        powf(az * G_PER_LSB, 2));

                accScaleSumCount++;

                if (accScaleSumCount == ACC_SCALE_SAMPLES) {
                    accScale = accScaleSum / ACC_SCALE_SAMPLES;
                    accScaleFound = true;
                }
            }

            return accScaleFound;
        }


        static void runImuTask(void *obj)
        {
            ((ImuTask *)obj)->run();
        }

        void run(void)
        {
            systemWaitStart();

            consolePrintf("IMU: starting loop\n");

            while (true) {

                Axis3f gyroScaledIMU;
                Axis3f accScaledIMU;
                Axis3f accScaled;

                if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {

                    data.interruptTimestamp = interruptTimestamp;

                    // Get data from chosen sensors 
                    readGyro(&gyroRaw);
                    readAccel(&accelRaw);

                    // Calibrate if necessary */
                    gyroBiasFound = processGyroBias(xTaskGetTickCount(),
                            gyroRaw.x, gyroRaw.y, gyroRaw.z, 
                            &gyroBias);

                    // Gyro
                    gyroScaledIMU.x =  
                        (gyroRaw.x - gyroBias.x) * DEG_PER_LSB;
                    gyroScaledIMU.y =  
                        (gyroRaw.y - gyroBias.y) * DEG_PER_LSB;
                    gyroScaledIMU.z =  
                        (gyroRaw.z - gyroBias.z) * DEG_PER_LSB;

                    alignToAirframe(&gyroScaledIMU, &data.gyro);
                    applyGyroLpf(&data.gyro);
                    _estimatorTask->enqueueGyro(&data.gyro, hal_isInInterrupt());

                    // Acelerometer
                    accScaledIMU.x = accelRaw.x * G_PER_LSB / accScale;
                    accScaledIMU.y = accelRaw.y * G_PER_LSB / accScale;
                    accScaledIMU.z = accelRaw.z * G_PER_LSB / accScale;

                    alignToAirframe(&accScaledIMU, &accScaled);
                    accAlignToGravity(&accScaled, &data.acc);
                    applyAccelLpf(&data.acc);
                    _estimatorTask->enqueueAccel(&data.acc, hal_isInInterrupt());
                }

                xQueueOverwrite(accelQueue, &data.acc);
                xQueueOverwrite(gyroQueue, &data.gyro);

                xSemaphoreGive(dataReady);
            }
        }

        // Hardware-dependent
        bool gyroSelfTest();
        void deviceInit(void); 
        void readGyro(Axis3i16* dataOut);
        void readAccel(Axis3i16* dataOut);
};
