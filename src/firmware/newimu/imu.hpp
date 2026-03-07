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

#include <datatypes.hpp>
#include <firmware/estimators/newekf.hpp>
#include <firmware/newimu/gyro_bias.hpp>
#include <firmware/newimu/three_axis_lpf.hpp>
#include <firmware/timer.hpp>
#include <num.hpp>

#define NEW

namespace hf {

    class IMU {

        private:

            static constexpr float CALIBRATION_PITCH = 0;
            static constexpr float CALIBRATION_ROLL = 0;

        public:

            IMU& operator=(const IMU& other) = default;

            IMU() = default;

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
                    const axis3_i16_t gyroRaw, 
                    const axis3_i16_t accelRaw,
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
                _gyroSamplesBuffer[_gyroBiasCalculator.bufferIndex] = gyroRaw;
                GyroBiasCalculator::process(
                        _gyroBiasCalculator,
                        _gyroSamplesBuffer,
                        tickCount);

                _gyroBias = _gyroBiasCalculator.biasOut;

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

                _accelLpf = _accelLpf.apply(
                        _accelLpf, accelAlignedToGravity, ACCEL_LPF_CUTOFF_FREQ);

                const auto accelFiltered = _accelLpf.output;

                gyroDps.x = gyroFiltered.x;
                gyroDps.y = gyroFiltered.y;
                gyroDps.z = gyroFiltered.z;

                accelGs.x = accelFiltered.x;
                accelGs.y = accelFiltered.y;
                accelGs.z = accelFiltered.z;

                return _gyroBiasCalculator.wasValueFound;
            }

        private:

            static const uint32_t ACC_SCALE_SAMPLES = 200;

            // IMU alignment on the airframe 
            static constexpr float ALIGN_PHI   = 0;
            static constexpr float ALIGN_THETA = 0;
            static constexpr float ALIGN_PSI   = 0;

            static constexpr float GYRO_LPF_CUTOFF_FREQ  = 80;
            static constexpr float ACCEL_LPF_CUTOFF_FREQ = 30;

            // ---------------------------------------------------------------

            axis3_i16_t _gyroSamplesBuffer[GyroBiasCalculator::NBR_OF_SAMPLES];

            GyroBiasCalculator _gyroBiasCalculator;

            ThreeAxisLpf _accelLpf;
            ThreeAxisLpf _gyroLpf;

            Vec3 _gyroBias;

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
