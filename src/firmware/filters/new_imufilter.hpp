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
#include <firmware/filters/new_three_axis_lpf.hpp>
#include <firmware/timer.hpp>
#include <num.hpp>

namespace hf {

    class NewImuFilter {

        private:

            static const uint32_t ACC_SCALE_SAMPLES = 200;

            static constexpr float GYRO_LPF_CUTOFF_FREQ  = 80;
            static constexpr float ACCEL_LPF_CUTOFF_FREQ = 30;

            static constexpr float GYRO_RAW_VARIANCE_BASE = 100;
            static const uint32_t GYRO_MIN_BIAS_TIMEOUT_MS = 1000;

            // Number of samples used in variance calculation. Changing this
            // affects the threshold
            static const uint16_t GYRO_NBR_OF_SAMPLES = 512;

        public:

            NewImuFiltered output;

            bool wasGyroBiasFound;

            NewImuFilter& operator=(const NewImuFilter& other) = default;

            NewImuFilter() = default;

            NewImuFilter(
                    const NewImuFiltered & output,
                    const bool wasGyroBiasFound,
                    const NewVec3 & gyroSum,
                    const NewVec3 & gyroSumOfSquares,
                    const uint16_t  gyroSampleCount,
                    const NewVec3 & gyroBias,
                    const uint32_t gyroVarianceSampleTimeMsec,
                    const NewThreeAxisLpf & accelLpf,
                    const NewThreeAxisLpf & gyroLpf) 
                : 
                    output(output),
                    wasGyroBiasFound(wasGyroBiasFound),
                    _gyroSum(gyroSum),
                    _gyroSumOfSquares(gyroSumOfSquares),
                    _gyroSampleCount(gyroSampleCount),
                    _gyroBias(gyroBias),
                    _gyroVarianceSampleTimeMsec(gyroVarianceSampleTimeMsec),
                    _accelLpf(accelLpf),
                    _gyroLpf(gyroLpf) {}

            /**
             * imuraw should come in as follows:
             *   gyro.x: positive roll-rightward
             *   gyro.y: positive nose-downward
             *   gyro.z: positive counter-clockwise
             *   accel.x: positive nose-up
             *   accel.y: positive roll-right
             *   accel.z: positive rightside-up
             */
            static auto step(
                    const NewImuFilter & filter,
                    const uint32_t msec_curr,
                    const ImuRaw & imuraw,
                    const int16_t gyro_range_dps,
                    const int16_t accel_range_gs)-> NewImuFilter
            {
                const auto gyroraw = imuraw.gyro;

                const auto gyromean = filter._gyroSum / GYRO_NBR_OF_SAMPLES;

                const auto gyrovariance =
                    filter._gyroSumOfSquares/GYRO_NBR_OF_SAMPLES -
                    square(gyromean);

                const auto gyroval = raw2float(gyroraw);

                const auto gyroSum = filter.wasGyroBiasFound ?
                    filter._gyroSum : filter._gyroSum + gyroval;

                const auto gyroSumOfSquares = filter.wasGyroBiasFound ?
                    filter._gyroSumOfSquares :
                    filter._gyroSumOfSquares + square(gyroval);

                const auto accel = scale(raw2float(imuraw.accel), accel_range_gs);

                const auto newBufferIndex = filter._gyroSampleCount + 1;

                const auto isBufferFilled =
                    newBufferIndex == GYRO_NBR_OF_SAMPLES;

                const auto wantUpdate =!filter.wasGyroBiasFound &&
                    isBufferFilled;

                const auto shouldUpdate = wantUpdate && 
                    lt(gyrovariance, GYRO_RAW_VARIANCE_BASE) &&
                    filter._gyroVarianceSampleTimeMsec +
                    GYRO_MIN_BIAS_TIMEOUT_MS < msec_curr;

                const auto gyroBias = shouldUpdate ?  gyromean :
                    filter._gyroBias;

                const auto gyroVarianceSampleTimeMsec =
                    shouldUpdate ? msec_curr : filter._gyroVarianceSampleTimeMsec;

                const auto wasGyroBiasFound = shouldUpdate ? true :
                    filter.wasGyroBiasFound;

                const auto gyroSampleCount = isBufferFilled ? 0 : newBufferIndex;

                const auto gyroUnbiased =
                    scale(gyroval - gyroBias, gyro_range_dps);

                const auto gyroLpf = filter._gyroLpf.apply(
                        filter._gyroLpf, gyroUnbiased, GYRO_LPF_CUTOFF_FREQ);

                const auto gyroFiltered = gyroLpf.output;

                const auto accelLpf = filter._accelLpf.apply(filter._accelLpf,
                        accel, ACCEL_LPF_CUTOFF_FREQ);

                const auto accelFiltered = filter._accelLpf.output;

                const auto output = NewImuFiltered(gyroFiltered, accelFiltered);

                return NewImuFilter(
                        output,
                        wasGyroBiasFound,
                        gyroSum,
                        gyroSumOfSquares,
                        gyroSampleCount,
                        gyroBias,
                        gyroVarianceSampleTimeMsec,
                        accelLpf,
                        gyroLpf);
            }

        private:

            NewVec3 _gyroSum;
            NewVec3 _gyroSumOfSquares;
            uint16_t _gyroSampleCount;
            NewVec3 _gyroBias;
            uint32_t _gyroVarianceSampleTimeMsec;
            NewThreeAxisLpf _accelLpf;
            NewThreeAxisLpf _gyroLpf;

            static auto lt(const NewVec3 & vec, const float val) -> bool
            {
                return vec(0) < val && vec(1) < val && vec(2) < val;
            }

            static auto raw2float(const Vec3Raw & raw) -> NewVec3
            {
                return NewVec3(raw.x, raw.y, raw.z);
            }

            static auto square(const NewVec3 & vec) -> NewVec3
            {
                return vec.cwiseProduct(vec);
            }

            static auto scale(const NewVec3 & vec, const int16_t s) -> NewVec3
            {
                return vec * 2 * (float)s / 65536;
            }
    };
}
