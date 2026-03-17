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

            static constexpr float GYRO_RAW_VARIANCE_BASE = 100;
            static const uint32_t GYRO_MIN_BIAS_TIMEOUT_MS = 1000;

            // Number of samples used in variance calculation. Changing this
            // affects the threshold
            static const uint16_t NBR_OF_SAMPLES = 512;

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

                const auto n = NBR_OF_SAMPLES;

                const auto gyromean = _gyrosum / n;

                const auto gyrovariance = (_gyrosumsq/n) - (gyromean*gyromean);

                const auto gyroval = Vec3(gyroraw.x, gyroraw.y, gyroraw.z);

                _gyrosum = wasGyroBiasFound ? _gyrosum : _gyrosum + gyroval;

                _gyrosumsq = wasGyroBiasFound ? _gyrosumsq :
                    _gyrosumsq + (gyroval * gyroval);

                // Convert accel to Gs
                const Vec3 accel = {
                    scale(imuraw.accel.x, accel_range_gs),
                    scale(imuraw.accel.y, accel_range_gs),
                    scale(imuraw.accel.z, accel_range_gs)
                };

                const auto newBufferIndex = _gyroSampleCount + 1;

                const auto isBufferFilled = newBufferIndex == NBR_OF_SAMPLES;

                const auto wantUpdate = !wasGyroBiasFound && isBufferFilled;

                if (wasGyroBiasFound) {

                    static bool _printed;

                    if (!_printed) {
                        printf("mean,%+3.3f,%+3.3f,%+3.3f\n", 
                                gyromean.x, gyromean.y, gyromean.z);
                        printf("variance,%+3.3f,%+3.3f,%+3.3f\n", 
                                gyrovariance.x, gyrovariance.y, gyrovariance.z);
                        _printed = true;
                    }
                }

                const auto shouldUpdate = wantUpdate &&
                    gyrovariance.x < GYRO_RAW_VARIANCE_BASE &&
                    gyrovariance.y < GYRO_RAW_VARIANCE_BASE &&
                    gyrovariance.z < GYRO_RAW_VARIANCE_BASE &&
                    _gyroVarianceSampleTimeMsec + GYRO_MIN_BIAS_TIMEOUT_MS < msec_curr;

                _gyroBias = shouldUpdate ?  gyromean : _gyroBias;

                _gyroVarianceSampleTimeMsec =
                    shouldUpdate ? msec_curr : _gyroVarianceSampleTimeMsec;

                wasGyroBiasFound = shouldUpdate ? true : wasGyroBiasFound;

                _gyroSampleCount = isBufferFilled ? 0 : newBufferIndex;

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
            }

        private:

            // ---------------------------------------------------------------

            Vec3 _gyrosum;
            Vec3 _gyrosumsq;
            uint16_t _gyroSampleCount;
            Vec3 _gyroBias;
            int32_t _gyroVarianceSampleTimeMsec;
            Vec3 _sumvals;
            ThreeAxisLpf _accelLpf;
            ThreeAxisLpf _gyroLpf;

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
