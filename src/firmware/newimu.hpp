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
#include <firmware/timer.hpp>
#include <num.hpp>

#define NEW

namespace hf {

    class IMU {

        private:

            static constexpr float CALIBRATION_PITCH = 0;
            static constexpr float CALIBRATION_ROLL = 0;

            class LPF {

                public:

                    float output;

                    LPF() : output(0), _delay1(0), _delay2(0) {}

                    LPF(const float output, const float delay1, const float delay2)
                        : output(output), _delay1(0), _delay2(0) {}

                    LPF& operator=(const LPF& other) = default;

                    static auto apply(
                            const LPF & lpf,
                            const float sample,
                            const float cutoff_freq,
                            const float sample_freq=1000) -> LPF
                    {
                        const auto fr = sample_freq/cutoff_freq;
                        const auto ohm = tanf(M_PI/fr);
                        const auto c = 1+2*cosf(M_PI/4)*ohm+ohm*ohm;

                        const auto b0 = ohm*ohm/c;
                        const auto b1 = 2*b0;
                        const auto a1 = 2*(ohm*ohm-1)/c;
                        const auto a2 = (1-2*cosf(M_PI/4)*ohm+ohm*ohm)/c;

                        const auto try_delay0 = sample - lpf._delay1 * a1 - lpf._delay2 * a2;

                        // don't allow bad values to propigate through the filter
                        const auto delay0 = isfinite(try_delay0) ? try_delay0 : sample;

                        const auto output = delay0 * b0 + lpf._delay1 * b1 + lpf._delay2 * b0;

                        return LPF(delay0, lpf._delay1, output);
                    }

                private:

                    float _delay1;
                    float _delay2;

            }; // class Lpf

            class ThreeAxisLpf {

                public:

                    Vec3 output;

                    ThreeAxisLpf() = default;

                    ThreeAxisLpf(const Vec3 & output,
                            const LPF &x, const LPF &y, const LPF &z)
                        : output(output), _x(x), _y(y), _z(z) {}

                    ThreeAxisLpf& operator=(const ThreeAxisLpf& other) = default;

                    static auto apply(const ThreeAxisLpf & lpf, const Vec3 & in,
                            const float cutoff_freq) -> ThreeAxisLpf
                    {
                        const auto x = LPF::apply(lpf._x, in.x, cutoff_freq);
                        const auto y = LPF::apply(lpf._y, in.y, cutoff_freq);
                        const auto z = LPF::apply(lpf._z, in.z, cutoff_freq);

                        const auto output = Vec3(x.output, y.output, z.output);

                        return ThreeAxisLpf(output, x, y, z);
                    }

                private:

                    LPF _x;
                    LPF _y;
                    LPF _z;

            }; // class ThreeAxisLpf

            class ThreeAxisStats {

                public:

                    Vec3 mean;
                    Vec3 variance;

                    ThreeAxisStats() : mean(0, 0, 0), variance(0, 0, 0) {}

                    ThreeAxisStats(const Vec3 & mean, const Vec3 & variance)
                        : mean(mean), variance(variance) {}

                    ThreeAxisStats& operator=(const ThreeAxisStats& other) = default;
            };

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

                _gyroLpf = _gyroLpf.apply(_gyroLpf, gyroAligned, GYRO_LPF_CUTOFF_FREQ);

                const auto gyroFiltered = _gyroLpf.output;

                const auto accelAlignedToAirframe = alignToAirframe(accel);

                const auto accelAlignedToGravity = alignToGravity(
                        accelAlignedToAirframe);

                _accelLpf = _accelLpf.apply(_accelLpf, accelAlignedToGravity, ACCEL_LPF_CUTOFF_FREQ);

                const auto accelFiltered = _accelLpf.output;

                gyroDps.x = gyroFiltered.x;
                gyroDps.y = gyroFiltered.y;
                gyroDps.z = gyroFiltered.z;

                accelGs.x = accelFiltered.x;
                accelGs.y = accelFiltered.y;
                accelGs.z = accelFiltered.z;

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

            class Bias {

                public:

                    Vec3 bias;

                    ThreeAxisStats stats;

                    Vec3 variance;
                    Vec3 mean;

                    bool isBiasValueFound;
                    bool isBufferFilled;
                    Axis3i16 * bufHead;
                    Axis3i16 buffer[NBR_OF_BIAS_SAMPLES];
            };

            // ---------------------------------------------------------------

            Bias _gyroBiasRunning;

            ThreeAxisLpf _accelLpf;
            ThreeAxisLpf _gyroLpf;

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
                    Bias & gyroBiasRunning)
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
                    Bias & gyroBiasRunning)
            {
                static int32_t varianceSampleTime;

                if (gyroBiasRunning.isBufferFilled)
                {
                    calculateVarianceAndMean(gyroBiasRunning,
                            gyroBiasRunning.variance, gyroBiasRunning.mean);

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

            static auto calculateVarianceAndMean(const Bias & bias) 
                -> ThreeAxisStats
            {
                int64_t sum[3] = {};
                int64_t sumSq[3] = {};

                for (uint16_t i=0; i<NBR_OF_BIAS_SAMPLES; i++) {

                    sum[0] += bias.buffer[i].x;
                    sum[1] += bias.buffer[i].y;
                    sum[2] += bias.buffer[i].z;
                    sumSq[0] += bias.buffer[i].x * bias.buffer[i].x;
                    sumSq[1] += bias.buffer[i].y * bias.buffer[i].y;
                    sumSq[2] += bias.buffer[i].z * bias.buffer[i].z;
                }

                const auto mean = Vec3(
                        (float) sum[0] / NBR_OF_BIAS_SAMPLES,
                        (float) sum[1] / NBR_OF_BIAS_SAMPLES,
                        (float) sum[2] / NBR_OF_BIAS_SAMPLES);

                const auto variance = Vec3(
                        sumSq[0] / NBR_OF_BIAS_SAMPLES - mean.x * mean.x,
                        sumSq[1] / NBR_OF_BIAS_SAMPLES - mean.y * mean.y,
                        sumSq[2] / NBR_OF_BIAS_SAMPLES - mean.z * mean.z);

                return ThreeAxisStats(mean, variance);
            }

            static void calculateVarianceAndMean(
                    const Bias & bias, Vec3 & varOut, Vec3 & meanOut)
            {
                int64_t sum[3] = {};
                int64_t sumSq[3] = {};

                for (uint16_t i=0; i<NBR_OF_BIAS_SAMPLES; i++) {

                    sum[0] += bias.buffer[i].x;
                    sum[1] += bias.buffer[i].y;
                    sum[2] += bias.buffer[i].z;
                    sumSq[0] += bias.buffer[i].x * bias.buffer[i].x;
                    sumSq[1] += bias.buffer[i].y * bias.buffer[i].y;
                    sumSq[2] += bias.buffer[i].z * bias.buffer[i].z;
                }

                meanOut.x = (float) sum[0] / NBR_OF_BIAS_SAMPLES;
                meanOut.y = (float) sum[1] / NBR_OF_BIAS_SAMPLES;
                meanOut.z = (float) sum[2] / NBR_OF_BIAS_SAMPLES;

                varOut.x =
                    sumSq[0] / NBR_OF_BIAS_SAMPLES - meanOut.x * meanOut.x;
                varOut.y =
                    sumSq[1] / NBR_OF_BIAS_SAMPLES - meanOut.y * meanOut.y;
                varOut.z =
                    sumSq[2] / NBR_OF_BIAS_SAMPLES - meanOut.z * meanOut.z;
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
