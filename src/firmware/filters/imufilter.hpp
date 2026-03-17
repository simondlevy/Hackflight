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

#include <firmware/datatypes.hpp>
#include <firmware/filters/three_axis_lpf.hpp>
#include <firmware/timer.hpp>
#include <num.hpp>

namespace hf {

    class ImuFilter {

        private:

            static constexpr float CALIBRATION_PITCH = 0;
            static constexpr float CALIBRATION_ROLL = 0;

            static const uint32_t ACC_SCALE_SAMPLES = 200;

            // IMU alignment on the airframe 
            static constexpr float ALIGN_PHI   = 0;
            static constexpr float ALIGN_THETA = 0;
            static constexpr float ALIGN_PSI   = 0;

            static constexpr float GYRO_LPF_CUTOFF_FREQ  = 80;
            static constexpr float ACCEL_LPF_CUTOFF_FREQ = 30;

            static constexpr float RAW_VARIANCE_BASE = 100;
            static const uint32_t MIN_BIAS_TIMEOUT_MS = 1000;

            // Number of samples used in variance calculation. Changing this
            // affects the threshold
            static const uint16_t NBR_OF_SAMPLES = 512;

            class GyroBiasCalculator {

                public:


                    Vec3 biasOut;
                    bool wasValueFound;
                    uint16_t bufferIndex;
                    ThreeAxisStats _stats;
                    Vec3 _values;
                    bool _isBufferFilled;
                    int32_t _varianceSampleTime;

                    GyroBiasCalculator() = default;

                    /*
                    GyroBiasCalculator& operator=(const GyroBiasCalculator& other)
                        = default;

                    GyroBiasCalculator(
                            const Vec3 & biasOut,
                            const bool wasValueFound,
                            const uint16_t bufferIndex,
                            const ThreeAxisStats & stats,
                            const Vec3 & values,
                            const bool isBufferFilled,
                            const int32_t varianceSampleTime)
                        :                     
                            biasOut(biasOut),
                            wasValueFound(wasValueFound),
                            bufferIndex(bufferIndex),
                            _stats(stats),
                            _values(values),
                            _isBufferFilled(isBufferFilled),
                            _varianceSampleTime(varianceSampleTime) {}*/

                    void process(const Vec3Raw * buffer, const uint32_t ticks)
                    {
                        const auto newBufferIndex = bufferIndex + 1;

                        _isBufferFilled = newBufferIndex == NBR_OF_SAMPLES;

                        const auto wantUpdate = !wasValueFound && _isBufferFilled;

                        _stats = wantUpdate ? calculateStats(buffer) : _stats;

                        const auto shouldUpdate = wantUpdate &&
                            _stats.variance.x < RAW_VARIANCE_BASE &&
                            _stats.variance.y < RAW_VARIANCE_BASE &&
                            _stats.variance.z < RAW_VARIANCE_BASE &&
                            _varianceSampleTime + MIN_BIAS_TIMEOUT_MS < ticks;

                        _values = shouldUpdate ? _stats.mean : _values;

                        biasOut = _values;

                        _varianceSampleTime =
                            shouldUpdate ? ticks : _varianceSampleTime;

                        wasValueFound = shouldUpdate ? true : wasValueFound;

                        bufferIndex = _isBufferFilled ? 0 : newBufferIndex;
                    }

                private:

                    static auto calculateStats(const Vec3Raw * buffer)
                        -> ThreeAxisStats
                        {
                            int64_t xsum=0, ysum=0, zsum=0;
                            int64_t xsumsq=0, ysumsq=0, zsumsq=0;

                            for (uint16_t i=0; i<NBR_OF_SAMPLES; i++) {

                                printf("values,%d,%d,%d\n", 
                                        buffer[i].x, buffer[i].y, buffer[i].z);

                                xsum += buffer[i].x;
                                ysum += buffer[i].y;
                                zsum += buffer[i].z;
                                xsumsq += buffer[i].x * buffer[i].x;
                                ysumsq += buffer[i].y * buffer[i].y;
                                zsumsq += buffer[i].z * buffer[i].z;
                            }

                            const auto mean = Vec3(
                                    (float) xsum / NBR_OF_SAMPLES,
                                    (float) ysum / NBR_OF_SAMPLES,
                                    (float) zsum / NBR_OF_SAMPLES);

                            const auto variance = Vec3(
                                    (float)xsumsq / NBR_OF_SAMPLES - mean.x * mean.x,
                                    (float)ysumsq / NBR_OF_SAMPLES - mean.y * mean.y,
                                    (float)zsumsq / NBR_OF_SAMPLES - mean.z * mean.z);

                            printf("\noldmean,%f,%f,%f\n",
                                    mean.x, mean.y, mean.z);
                            printf("oldvariance,%f,%f,%f\n\n",
                                    variance.x, variance.y, variance.z);

                            return ThreeAxisStats(mean, variance);
                        }

            }; // class GyroBias

        public:

            ImuFiltered output;

            bool wasGyroBiasFound;

            ImuFilter& operator=(const ImuFilter& other) = default;

            ImuFilter() = default;

            void step(
                    const uint32_t msec_curr,
                    const ImuRaw & imuraw,
                    const int16_t gyro_range_dps,
                    const int16_t accel_range_gs)
            {
                const auto gyroraw = imuraw.gyro;

                static bool _printed;
                if (wasGyroBiasFound) {

                    const auto n = NBR_OF_SAMPLES;

                    const auto gyromean = _gyrosum / n;

                    const auto gyrovariance = (_gyrosumsq/n) - (gyromean*gyromean);

                    if (!_printed) {
                        printf("newmean,%f,%f,%f\n", 
                                gyromean.x, gyromean.y, gyromean.z);
                        printf("newvariance,%f,%f,%f\n", 
                                gyrovariance.x, gyrovariance.y, gyrovariance.z);
                        _printed = true;
                    }
                }
                else {
                    const auto gyroval = Vec3(gyroraw.x, gyroraw.y, gyroraw.z);
                    _gyrosum = _gyrosum + gyroval;
                    _gyrosumsq = _gyrosumsq + (gyroval * gyroval);
                }

                // Convert accel to Gs
                const Vec3 accel = {
                    scale(imuraw.accel.x, accel_range_gs),
                    scale(imuraw.accel.y, accel_range_gs),
                    scale(imuraw.accel.z, accel_range_gs)
                };

                // Calibrate gyro with raw values if necessary
                _gyroSamplesBuffer[_gyroBiasCalculator.bufferIndex] = gyroraw;

                _gyroBiasCalculator.process(_gyroSamplesBuffer, msec_curr);

                _gyroBias = _gyroBiasCalculator.biasOut;

                // Subtract gyro bias
                const Vec3 gyroUnbiased = {
                    scale(imuraw.gyro.x - _gyroBias.x, gyro_range_dps),
                    scale(imuraw.gyro.y - _gyroBias.y, gyro_range_dps),
                    scale(imuraw.gyro.z - _gyroBias.z, gyro_range_dps)
                };

                const auto gyroAligned = alignToAirframe(gyroUnbiased);

                _gyroLpf = _gyroLpf.apply(_gyroLpf, gyroAligned, GYRO_LPF_CUTOFF_FREQ);

                const auto gyroFiltered = _gyroLpf.output;

                const auto accelAlignedToAirframe = alignToAirframe(accel);

                const auto accelAlignedToGravity = alignToGravity(
                        accelAlignedToAirframe);

                _accelLpf = _accelLpf.apply(
                        _accelLpf, accelAlignedToGravity, ACCEL_LPF_CUTOFF_FREQ);

                const auto accelFiltered = _accelLpf.output;

                output.gyroDps.x = gyroFiltered.x;
                output.gyroDps.y = gyroFiltered.y;
                output.gyroDps.z = gyroFiltered.z;

                output.accelGs.x = accelFiltered.x;
                output.accelGs.y = accelFiltered.y;
                output.accelGs.z = accelFiltered.z;

                wasGyroBiasFound = _gyroBiasCalculator.wasValueFound;
            }

        private:

            // ---------------------------------------------------------------

            Vec3 _gyrosum;
            Vec3 _gyrosumsq;

            GyroBiasCalculator _gyroBiasCalculator;

            Vec3 _sumvals;

            ThreeAxisLpf _accelLpf;
            ThreeAxisLpf _gyroLpf;

            Vec3 _gyroBias;

            Vec3Raw _gyroSamplesBuffer[NBR_OF_SAMPLES];

            // ---------------------------------------------------------------

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
