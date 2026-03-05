/* Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy * * This program
 * is free software: you can redistribute it and/or modify * it under the terms
 * of the GNU General Public License as published by * the Free Software
 * Foundation, in version 3.
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
#include <firmware/estimators/newekf.hpp>
#include <firmware/lpf.hpp>
#include <firmware/timer.hpp>
#include <num.hpp>

#define NEW

namespace hf {

    class IMU {

        private:

            static constexpr float CALIBRATION_PITCH = 0;
            static constexpr float CALIBRATION_ROLL = 0;

        public:

            typedef struct {
                int16_t x;
                int16_t y;
                int16_t z;
            } Axis3i16;

            IMU& operator=(const IMU& other) = default;

            IMU()
            {
                _gyroBiasRunning.isBufferFilled = false;

                _gyroBiasRunning.bufHead = _gyroBiasRunning.buffer;

                for (uint8_t i = 0; i < 3; i++) {
                    _gyroLpf[i].init(1000, GYRO_LPF_CUTOFF_FREQ);
                    _accLpf[i].init(1000, ACCEL_LPF_CUTOFF_FREQ);
                }
            }

            /**
             * gyro.x: positive roll-rightward
             * gyro.y: positive nose-downward
             * gyro.z: positive counter-clockwise
             * accel.x: positive nose-up
             * accel.y: positive roll-right
             * accel.z: positive rightside-up
             */
            bool step(
                    const uint32_t tickCount,
                    const Axis3i16 gyroRaw, 
                    const Axis3i16 accelRaw,
                    const int16_t gscale,
                    const int16_t ascale,
                    Vec3 & gyroDps,
                    Vec3 & accelGs)
            {

                // Convert accel to Gs
                const Vec3 accel = {
                    scale(accelRaw.x, ascale),
                    scale(accelRaw.y, ascale),
                    scale(accelRaw.z, ascale)
                };

                // Calibrate gyro with raw values if necessary
                const auto gyroBiasFound = processGyroBias(tickCount, gyroRaw,
                        _gyroBias, _gyroBiasRunning);

                // Subtract gyro bias
                const Vec3 gyroUnbiased = {
                    scale(gyroRaw.x - _gyroBias.x, gscale),
                    scale(gyroRaw.y - _gyroBias.y, gscale),
                    scale(gyroRaw.z - _gyroBias.z, gscale)
                };

                const auto gyroAligned = alignToAirframe(gyroUnbiased);

                applyLpf(_gyroLpf, gyroAligned, gyroDps);

                const auto accelAlignedToAirframe = alignToAirframe(accel);

                const auto accelAlignedToGravity = alignToGravity(
                        accelAlignedToAirframe);

                applyLpf(_accLpf, accelAlignedToGravity, accelGs);

                return gyroBiasFound;
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

            typedef struct {

                Vec3     bias;
                Vec3     variance;
                Vec3     mean;
                bool       isBiasValueFound;
                bool       isBufferFilled;
                Axis3i16*  bufHead;
                Axis3i16   buffer[NBR_OF_BIAS_SAMPLES];

            } bias_t;

            // ---------------------------------------------------------------

            bias_t _gyroBiasRunning;

            LPF _accLpf[3];
            LPF _gyroLpf[3];

            Vec3 _gyroBias;

            // ---------------------------------------------------------------

            /**
             * Calculates the bias first when the gyro variance is below threshold.
             * Requires a buffer but calibrates platform first when it is stable.
             */
            static bool processGyroBias(
                    const uint32_t tickCount,
                    const Axis3i16 gyroRaw,
                    Vec3 & gyroBiasOut,
                    bias_t & gyroBiasRunning)
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
                    findBiasValue(tickCount, gyroBiasRunning);
                }

                gyroBiasOut.x = gyroBiasRunning.bias.x;
                gyroBiasOut.y = gyroBiasRunning.bias.y;
                gyroBiasOut.z = gyroBiasRunning.bias.z;

                return gyroBiasRunning.isBiasValueFound;
            }

            /**
             * Checks if the variances is below the predefined thresholds.
             * The bias value should have been added before calling this.
             * @param bias  The bias object
             */
            static void findBiasValue(
                    const uint32_t ticks,
                    bias_t & gyroBiasRunning)
            {
                static int32_t varianceSampleTime;

                if (gyroBiasRunning.isBufferFilled)
                {
                    calculateVarianceAndMean( &gyroBiasRunning,
                            &gyroBiasRunning.variance, &gyroBiasRunning.mean);

                    if (gyroBiasRunning.variance.x < RAW_GYRO_VARIANCE_BASE &&
                            gyroBiasRunning.variance.y < RAW_GYRO_VARIANCE_BASE &&
                            gyroBiasRunning.variance.z < RAW_GYRO_VARIANCE_BASE &&
                            (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < ticks))
                    {
                        varianceSampleTime = ticks;
                        gyroBiasRunning.bias.x = gyroBiasRunning.mean.x;
                        gyroBiasRunning.bias.y = gyroBiasRunning.mean.y;
                        gyroBiasRunning.bias.z = gyroBiasRunning.mean.z;
                        gyroBiasRunning.isBiasValueFound = true;
                    }
                }
            }

            /**
             * Compensate for a miss-aligned accelerometer. It uses the trim
             * data gathered from the UI and written in the config-block to
             * rotate the accelerometer to be aligned with gravity.
             */
            static auto alignToGravity(const Vec3 & in) -> Vec3
            {

                const auto cosPitch = cosf(CALIBRATION_PITCH * Num::DEG2RAD);
                const auto sinPitch = sinf(CALIBRATION_PITCH * Num::DEG2RAD);
                const auto cosRoll = cosf(CALIBRATION_ROLL * Num::DEG2RAD);
                const auto sinRoll = sinf(CALIBRATION_ROLL * Num::DEG2RAD);

                // Rotate around x-axis
                const Vec3 rx = {
                    in.x,
                    in.y * cosRoll - in.z * sinRoll,
                    in.y * sinRoll + in.z * cosRoll
                };

                // Rotate around y-axis
                return Vec3(
                        rx.x * cosPitch - rx.z * sinPitch,
                        rx.y,
                        -rx.x * sinPitch + rx.z * cosPitch);
            }

            static void calculateVarianceAndMean(
                    bias_t* bias, Vec3* varOut, Vec3* meanOut)
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

            static void applyLpf(LPF lpf[3], Vec3 & in)
            {
                in.x = lpf[0].apply(in.x);
                in.y = lpf[1].apply(in.y);
                in.z = lpf[2].apply(in.z);
            }

            static void applyLpf(LPF lpf[3], const Vec3 & in, Vec3 & out)
            {
                out.x = lpf[0].apply(in.x);
                out.y = lpf[1].apply(in.y);
                out.z = lpf[2].apply(in.z);
            }

            static auto alignToAirframe(const Vec3 & in) -> Vec3
            {
                const auto sphi   = sinf(ALIGN_PHI * Num::DEG2RAD);
                const auto cphi   = cosf(ALIGN_PHI * Num::DEG2RAD);
                const auto stheta = sinf(ALIGN_THETA * Num::DEG2RAD);
                const auto ctheta = cosf(ALIGN_THETA * Num::DEG2RAD);
                const auto spsi   = sinf(ALIGN_PSI * Num::DEG2RAD);
                const auto cpsi   = cosf(ALIGN_PSI * Num::DEG2RAD);

                const auto r00 = ctheta * cpsi;
                const auto r01 = ctheta * spsi;
                const auto r02 = -stheta;
                const auto r10 = sphi * stheta * cpsi - cphi * spsi;
                const auto r11 = sphi * stheta * spsi + cphi * cpsi;
                const auto r12 = sphi * ctheta;
                const auto r20 = cphi * stheta * cpsi + sphi * spsi;
                const auto r21 = cphi * stheta * spsi - sphi * cpsi;
                const auto r22 = cphi * ctheta;

                return Vec3(
                        in.x*r00 + in.y*r01 + in.z*r02,
                        in.x*r10 + in.y*r11 + in.z*r12,
                        in.x*r20 + in.y*r21 + in.z*r22);
            }

            static float scale(const int16_t raw, const int16_t scale)
            {
                return (float)raw * 2 * scale / 65536.f;
            }

    };

}
