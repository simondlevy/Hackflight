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

#include <firmware/imu/sensor.hpp>
#include <firmware/imu/three_axis.hpp>
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

            class Data {

                public:

                    ThreeAxis gyro_dps;
                    ThreeAxis accel_gs;

                    Data() = default;

                    Data(const ThreeAxis & gyro_dps, const ThreeAxis & accel_gs) 
                        : gyro_dps(gyro_dps), accel_gs(accel_gs) {}

                    Data& operator=(const Data& other) = default;
            };

            Data output;

            bool is_gyro_calibrated;

            ImuFilter& operator=(const ImuFilter& other) = default;

            ImuFilter() = default;

            ImuFilter(
                    const Data & output,
                    const bool is_gyro_calibrated,
                    const ThreeAxis & gyro_sum,
                    const ThreeAxis & gyro_sum_of_squares,
                    const uint16_t  gyro_sample_count,
                    const ThreeAxis & gyro_bias,
                    const uint32_t gyro_variance_sample_time_msec,
                    const ThreeAxisLpf & accel_lpf,
                    const ThreeAxisLpf & gyro_lpf) 
                : 
                    output(output),
                    is_gyro_calibrated(is_gyro_calibrated),
                    _gyro_sum(gyro_sum),
                    _gyro_sum_of_squares(gyro_sum_of_squares),
                    _gyro_sample_count(gyro_sample_count),
                    _gyro_bias(gyro_bias),
                    _gyro_variance_sample_time_msec(gyro_variance_sample_time_msec),
                    _accel_lpf(accel_lpf),
                    _gyro_lpf(gyro_lpf) {}

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

                const auto gyromean = filter._gyro_sum / GYRO_NBR_OF_SAMPLES;

                const auto gyrovariance =
                    (filter._gyro_sum_of_squares/GYRO_NBR_OF_SAMPLES) -
                    square(gyromean);

                const auto gyro_val = ThreeAxis(gyroraw.x, gyroraw.y, gyroraw.z);

                const auto gyro_sum = filter.is_gyro_calibrated ?
                    filter._gyro_sum : filter._gyro_sum + gyro_val;

                const auto gyro_sum_of_squares = filter.is_gyro_calibrated ?
                    filter._gyro_sum_of_squares :
                    filter._gyro_sum_of_squares + square(gyro_val);

                const auto accel_raw = imuraw.accel;
                const auto accel = scale(
                        ThreeAxis(accel_raw.x, accel_raw.y, accel_raw.z),
                        accel_range_gs);

                const auto new_buffer_index = filter._gyro_sample_count + 1;

                const auto is_buffer_filled =
                    new_buffer_index == GYRO_NBR_OF_SAMPLES;

                const auto want_update =!filter.is_gyro_calibrated &&
                    is_buffer_filled;

                const bool is_gyro_variance_low = gyrovariance < GYRO_RAW_VARIANCE_BASE;

                const bool in_sample_window = (filter._gyro_variance_sample_time_msec +
                        GYRO_MIN_BIAS_TIMEOUT_MS) < msec_curr;

                const auto should_update =
                    want_update && is_gyro_variance_low && in_sample_window;

                const auto gyro_bias = should_update ?  gyromean : filter._gyro_bias;

                const auto gyro_variance_sample_time_msec =
                    should_update ? msec_curr : filter._gyro_variance_sample_time_msec;

                const auto is_gyro_calibrated = should_update ? true :
                    filter.is_gyro_calibrated;

                const auto gyro_sample_count = is_buffer_filled ? 0 : new_buffer_index;

                const auto gyro_unbiased =
                    scale(gyro_val - gyro_bias, gyro_range_dps);

                const auto gyro_lpf = filter._gyro_lpf.apply(
                        filter._gyro_lpf, gyro_unbiased, GYRO_LPF_CUTOFF_FREQ);

                const auto gyro_filtered = gyro_lpf.output;

                const auto accel_lpf = filter._accel_lpf.apply(filter._accel_lpf,
                        accel, ACCEL_LPF_CUTOFF_FREQ);

                const auto accel_filtered = filter._accel_lpf.output;

                const auto output = Data(gyro_filtered, accel_filtered);

                return ImuFilter(output, is_gyro_calibrated, gyro_sum,
                        gyro_sum_of_squares, gyro_sample_count, gyro_bias,
                        gyro_variance_sample_time_msec, accel_lpf, gyro_lpf);
            }

        private:

            ThreeAxis _gyro_sum;
            ThreeAxis _gyro_sum_of_squares;
            uint16_t _gyro_sample_count;
            ThreeAxis _gyro_bias;
            uint32_t _gyro_variance_sample_time_msec;
            ThreeAxisLpf _accel_lpf;
            ThreeAxisLpf _gyro_lpf;

            static auto square(const ThreeAxis & vec) -> ThreeAxis
            {
                return vec * vec;
            }

            static auto scale(const ThreeAxis & vec, const int16_t s) -> ThreeAxis
            {
                return vec * 2 * (float)s / 65536;
            }
    };
}
