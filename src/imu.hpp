/** * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy * * This program is free software: you can redistribute it and/or modify * it under the terms of the GNU General Public License as published by * the Free Software Foundation, in version 3.
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

#include <datatypes.h>
#include <debugger.hpp>
#include <ekf.hpp>
#include <lpf.hpp>
#include <num.hpp>
#include <timer.hpp>

class Imu {

    private:

        static constexpr float CALIBRATION_PITCH = 0;
        static constexpr float CALIBRATION_ROLL = 0;

    public:

        void init()
        {
            if (!device_init(_gscale, _ascale)) {
                Debugger::error("IMU");
            }

            _gyroBiasRunning.isBufferFilled = false;
            _gyroBiasRunning.bufHead = _gyroBiasRunning.buffer;

            // Calibrate
            for (uint8_t i = 0; i < 3; i++) {
                _gyroLpf[i].init(1000, GYRO_LPF_CUTOFF_FREQ);
                _accLpf[i].init(1000, ACCEL_LPF_CUTOFF_FREQ);
            }
            _cosPitch = cosf(CALIBRATION_PITCH * (float) M_PI / 180);
            _sinPitch = sinf(CALIBRATION_PITCH * (float) M_PI / 180);
            _cosRoll = cosf(CALIBRATION_ROLL * (float) M_PI / 180);
            _sinRoll = sinf(CALIBRATION_ROLL * (float) M_PI / 180);
        }

        bool step(EKF * ekf, const uint32_t tickCount,
                Debugger * debugger=nullptr)
        {
            Axis3i16 gyroRaw = {};
            Axis3i16 accelRaw = {};

            device_read(
                    gyroRaw.x, gyroRaw.y, gyroRaw.z,
                    accelRaw.x, accelRaw.y, accelRaw.z);

            /*
            Debugger::printf(debugger,
                    "gx=%+4d gy=%+4d  gz=%+4d ax=%+4d ay=%+4d az=%+4d",
                    gyroRaw.x, gyroRaw.y, gyroRaw.z,
                    accelRaw.x, accelRaw.y, accelRaw.z);*/

            // Convert accel to Gs
            axis3_t accel = {
                scale(accelRaw.x, _ascale),
                scale(accelRaw.y, _ascale),
                scale(accelRaw.z, _ascale)
            };

            // Calibrate gyro with raw values if necessary
            _gyroBiasFound = processGyroBias(tickCount, gyroRaw, &_gyroBias);

            // Subtract gyro bias
            axis3_t gyroUnbiased = {
                scale(gyroRaw.x - _gyroBias.x, _gscale),
                scale(gyroRaw.y - _gyroBias.y, _gscale),
                scale(gyroRaw.z - _gyroBias.z, _gscale)
            };

            // Rotate gyro to airframe
            alignToAirframe(&gyroUnbiased, &_gyroData);

            // LPF gyro
            applyLpf(_gyroLpf, &_gyroData);

            axis3_t accelScaled = {};
            alignToAirframe(&accel, &accelScaled);

            axis3_t accelGs = {};

            accAlignToGravity(&accelScaled, &accelGs);

            applyLpf(_accLpf, &accelGs);

            ekf->enqueueImu(&_gyroData, &accelGs);

            return _gyroBiasFound;
        }

        void getGyroData(axis3_t & gyroData)
        {
            memcpy(&gyroData, &_gyroData, sizeof(axis3_t));
        }

    private:

        static constexpr float RAW_GYRO_VARIANCE_BASE = 100;

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

            axis3_t     bias;
            axis3_t     variance;
            axis3_t     mean;
            bool       isBiasValueFound;
            bool       isBufferFilled;
            Axis3i16*  bufHead;
            Axis3i16   buffer[NBR_OF_BIAS_SAMPLES];

        } bias_t;

        axis3_t _gyroData;  // deg/s

        static void calculateVarianceAndMean(
                bias_t* bias, axis3_t* varOut, axis3_t* meanOut)
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

        bias_t _gyroBiasRunning;

        EKF * _ekf;

        /**
         * Checks if the variances is below the predefined thresholds.
         * The bias value should have been added before calling this.
         * @param bias  The bias object
         */
        bool findBiasValue(const uint32_t ticks)
        {
            static int32_t varianceSampleTime;
            bool foundBias = false;

            if (_gyroBiasRunning.isBufferFilled)
            {
                calculateVarianceAndMean( &_gyroBiasRunning,
                        &_gyroBiasRunning.variance, &_gyroBiasRunning.mean);

                if (_gyroBiasRunning.variance.x < RAW_GYRO_VARIANCE_BASE &&
                        _gyroBiasRunning.variance.y < RAW_GYRO_VARIANCE_BASE &&
                        _gyroBiasRunning.variance.z < RAW_GYRO_VARIANCE_BASE &&
                        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < ticks))
                {
                    varianceSampleTime = ticks;
                    _gyroBiasRunning.bias.x = _gyroBiasRunning.mean.x;
                    _gyroBiasRunning.bias.y = _gyroBiasRunning.mean.y;
                    _gyroBiasRunning.bias.z = _gyroBiasRunning.mean.z;
                    foundBias = true;
                    _gyroBiasRunning.isBiasValueFound = true;
                }
            }

            return foundBias;
        }

        static void applyLpf(Lpf lpf[3], axis3_t* in)
        {
            in->x = lpf[0].apply(in->x);
            in->y = lpf[1].apply(in->y);
            in->z = lpf[2].apply(in->z);
        }

        // Low Pass filtering
        Lpf _accLpf[3];
        Lpf _gyroLpf[3];

        // Pre-calculated values for accelerometer alignment
        float _cosPitch;
        float _sinPitch;
        float _cosRoll;
        float _sinRoll;

        static void alignToAirframe(axis3_t* in, axis3_t* out)
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

        bool _gyroBiasFound;
        axis3_t _gyroBias;

        int16_t _gscale;
        int16_t _ascale;

        /**
         * Compensate for a miss-aligned accelerometer. It uses the trim
         * data gathered from the UI and written in the config-block to
         * rotate the accelerometer to be aligned with gravity.
         */
        void accAlignToGravity(axis3_t* in, axis3_t* out)
        {

            // Rotate around x-axis
            axis3_t rx = {};
            rx.x = in->x;
            rx.y = in->y * _cosRoll - in->z * _sinRoll;
            rx.z = in->y * _sinRoll + in->z * _cosRoll;

            // Rotate around y-axis
            axis3_t ry = {};
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
                const Axis3i16 gyroRaw, axis3_t *gyroBiasOut)
        {
            _gyroBiasRunning.bufHead->x = gyroRaw.x;
            _gyroBiasRunning.bufHead->y = gyroRaw.y;
            _gyroBiasRunning.bufHead->z = gyroRaw.z;
            _gyroBiasRunning.bufHead++;

            if (_gyroBiasRunning.bufHead >= 
                    &_gyroBiasRunning.buffer[NBR_OF_BIAS_SAMPLES]) {

                _gyroBiasRunning.bufHead = _gyroBiasRunning.buffer;
                _gyroBiasRunning.isBufferFilled = true;
            }

            if (!_gyroBiasRunning.isBiasValueFound) {
                findBiasValue(tickCount);
            }

            gyroBiasOut->x = _gyroBiasRunning.bias.x;
            gyroBiasOut->y = _gyroBiasRunning.bias.y;
            gyroBiasOut->z = _gyroBiasRunning.bias.z;

            return _gyroBiasRunning.isBiasValueFound;
        }

        static float scale(const int16_t raw, const int16_t scale)
        {
            return (float)raw * 2 * scale / 65536.f;
        }

        bool device_init(int16_t & gscale, int16_t & ascale);

        /**
          * gx: positive roll-rightward
          * gy: positive nose-downward
          * gz: positive counter-clockwise
          * ax: positive nose-up
          * ay: positive roll-right
          * az: positive rightside-up
          */
        void device_read(
                int16_t & gx, int16_t & gy, int16_t & gz,
                int16_t & ax, int16_t & ay, int16_t & az);
};
