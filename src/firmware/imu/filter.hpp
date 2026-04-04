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

#include <firmware/imu/imu.hpp>
#include <firmware/imu/three_axis_lpf.hpp>

namespace hf {

    class ImuFilter {

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

            IMU::ImuFiltered output;

            bool isGyroCalibrated;

            ImuFilter& operator=(const ImuFilter& other) = default;

            ImuFilter() = default;

            ImuFilter(
                    const IMU::ImuFiltered & output,
                    const bool isGyroCalibrated,
                    const IMU::ThreeAxis & gyroSum,
                    const IMU::ThreeAxis & gyroSumOfSquares,
                    const uint16_t  gyroSampleCount,
                    const IMU::ThreeAxis & gyroBias,
                    const uint32_t gyroVarianceSampleTimeMsec,
                    const ThreeAxisLpf & accelLpf,
                    const ThreeAxisLpf & gyroLpf) 
                : 
                    output(output),
                    isGyroCalibrated(isGyroCalibrated),
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
                    const ImuFilter & filter,
                    const uint32_t msec_curr,
                    const IMU::RawData & imuraw,
                    const int16_t gyro_range_dps,
                    const int16_t accel_range_gs)-> ImuFilter
            {
                const auto gyroraw = imuraw.gyro;

                const auto gyromean = filter._gyroSum / GYRO_NBR_OF_SAMPLES;

                const auto gyrovariance =
                    (filter._gyroSumOfSquares/GYRO_NBR_OF_SAMPLES) -
                    square(gyromean);

                const auto gyroval = IMU::ThreeAxis(gyroraw.x, gyroraw.y, gyroraw.z);

                const auto gyroSum = filter.isGyroCalibrated ?
                    filter._gyroSum : filter._gyroSum + gyroval;

                const auto gyroSumOfSquares = filter.isGyroCalibrated ?
                    filter._gyroSumOfSquares :
                    filter._gyroSumOfSquares + square(gyroval);

                const auto accelraw = imuraw.accel;
                const auto accel = scale(
                        IMU::ThreeAxis(accelraw.x, accelraw.y, accelraw.z),
                        accel_range_gs);

                const auto newBufferIndex = filter._gyroSampleCount + 1;

                const auto isBufferFilled =
                    newBufferIndex == GYRO_NBR_OF_SAMPLES;

                const auto wantUpdate =!filter.isGyroCalibrated &&
                    isBufferFilled;

                const bool isGyroVarianceLow = gyrovariance < GYRO_RAW_VARIANCE_BASE;

                const bool inSampleWindow = (filter._gyroVarianceSampleTimeMsec +
                     GYRO_MIN_BIAS_TIMEOUT_MS) < msec_curr;

                const auto shouldUpdate =
                    wantUpdate && isGyroVarianceLow && inSampleWindow;

                /*
                static bool _didUpdate;
                static uint32_t _count;
                if (shouldUpdate) {
                    _didUpdate = true;
                }
                if (!_didUpdate) {
                    printf("%05lu: (x=%+5.0f y=%+5.0f z=%+5.0f) => " 
                            "(x=%+5.0f y=%+5.0f z=%+5.0f)\n",
                            _count++,
                            gyroval.x, gyroval.y, gyroval.z,
                            gyrovariance.x, gyrovariance.y, gyrovariance.z);
                }*/

                const auto gyroBias = shouldUpdate ?  gyromean : filter._gyroBias;

                const auto gyroVarianceSampleTimeMsec =
                    shouldUpdate ? msec_curr : filter._gyroVarianceSampleTimeMsec;

                const auto isGyroCalibrated = shouldUpdate ? true :
                    filter.isGyroCalibrated;

                const auto gyroSampleCount = isBufferFilled ? 0 : newBufferIndex;

                const auto gyroUnbiased =
                    scale(gyroval - gyroBias, gyro_range_dps);

                const auto gyroLpf = filter._gyroLpf.apply(
                        filter._gyroLpf, gyroUnbiased, GYRO_LPF_CUTOFF_FREQ);

                const auto gyroFiltered = gyroLpf.output;

                const auto accelLpf = filter._accelLpf.apply(filter._accelLpf,
                        accel, ACCEL_LPF_CUTOFF_FREQ);

                const auto accelFiltered = filter._accelLpf.output;

                const auto output = IMU::ImuFiltered(gyroFiltered, accelFiltered);

                return ImuFilter(output, isGyroCalibrated, gyroSum,
                        gyroSumOfSquares, gyroSampleCount, gyroBias,
                        gyroVarianceSampleTimeMsec, accelLpf, gyroLpf);
            }

        private:

            IMU::ThreeAxis _gyroSum;
            IMU::ThreeAxis _gyroSumOfSquares;
            uint16_t _gyroSampleCount;
            IMU::ThreeAxis _gyroBias;
            uint32_t _gyroVarianceSampleTimeMsec;
            ThreeAxisLpf _accelLpf;
            ThreeAxisLpf _gyroLpf;

            static auto square(const IMU::ThreeAxis & vec) -> IMU::ThreeAxis
            {
                return vec * vec;
            }

            static auto scale(const IMU::ThreeAxis & vec, const int16_t s) -> IMU::ThreeAxis
            {
                return vec * 2 * (float)s / 65536;
            }
    };
}
