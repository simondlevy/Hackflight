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
                    gyro_sum_(gyro_sum),
                    gyro_sum_of_squares_(gyro_sum_of_squares),
                    gyro_sample_count_(gyro_sample_count),
                    gyro_bias_(gyro_bias),
                    gyro_variance_sample_time_msec_(gyro_variance_sample_time_msec),
                    accel_lpf_(accel_lpf),
                    gyro_lpf_(gyro_lpf) {}

            /**
             * imuraw should come in as follows:
             *   gyro.x: positive roll-rightward
             *   gyro.y: positive nose-downward
             *   gyro.z: positive counter-clockwise
             *   accel.x: positive nose-up
             *   accel.y: positive roll-right
             *   accel.z: positive rightside-up
             */
            static auto Step(
                    const ImuFilter & filter,
                    const uint32_t msec_curr,
                    const IMU::RawData & imuraw,
                    const int16_t gyro_range_dps,
                    const int16_t accel_range_gs)-> ImuFilter
            {
                const auto gyroraw = imuraw.gyro;

                const auto gyromean = filter.gyro_sum_ / GYRO_NBR_OF_SAMPLES;

                const auto gyrovariance =
                    (filter.gyro_sum_of_squares_/GYRO_NBR_OF_SAMPLES) -
                    Square(gyromean);

                const auto gyro_val = ThreeAxis(gyroraw.x, gyroraw.y, gyroraw.z);

                const auto gyro_sum = filter.is_gyro_calibrated ?
                    filter.gyro_sum_ : filter.gyro_sum_ + gyro_val;

                const auto gyro_sum_of_squares = filter.is_gyro_calibrated ?
                    filter.gyro_sum_of_squares_ :
                    filter.gyro_sum_of_squares_ + Square(gyro_val);

                const auto accel_raw = imuraw.accel;
                const auto accel = Scale(
                        ThreeAxis(accel_raw.x, accel_raw.y, accel_raw.z),
                        accel_range_gs);

                const auto new_buffer_index = filter.gyro_sample_count_ + 1;

                const auto is_buffer_filled =
                    new_buffer_index == GYRO_NBR_OF_SAMPLES;

                const auto want_update =!filter.is_gyro_calibrated &&
                    is_buffer_filled;

                const bool is_gyro_variance_low = gyrovariance < GYRO_RAW_VARIANCE_BASE;

                const bool in_sample_window = (filter.gyro_variance_sample_time_msec_ +
                        GYRO_MIN_BIAS_TIMEOUT_MS) < msec_curr;

                const auto should_update =
                    want_update && is_gyro_variance_low && in_sample_window;

                const auto gyro_bias = should_update ?  gyromean : filter.gyro_bias_;

                const auto gyro_variance_sample_time_msec =
                    should_update ? msec_curr : filter.gyro_variance_sample_time_msec_;

                const auto is_gyro_calibrated = should_update ? true :
                    filter.is_gyro_calibrated;

                const auto gyro_sample_count = is_buffer_filled ? 0 : new_buffer_index;

                const auto gyro_unbiased =
                    Scale(gyro_val - gyro_bias, gyro_range_dps);

                const auto gyro_lpf = filter.gyro_lpf_.apply(
                        filter.gyro_lpf_, gyro_unbiased, GYRO_LPF_CUTOFF_FREQ);

                const auto gyro_filtered = gyro_lpf.output;

                const auto accel_lpf = filter.accel_lpf_.apply(filter.accel_lpf_,
                        accel, ACCEL_LPF_CUTOFF_FREQ);

                const auto accel_filtered = filter.accel_lpf_.output;

                const auto output = Data(gyro_filtered, accel_filtered);

                return ImuFilter(output, is_gyro_calibrated, gyro_sum,
                        gyro_sum_of_squares, gyro_sample_count, gyro_bias,
                        gyro_variance_sample_time_msec, accel_lpf, gyro_lpf);
            }

        private:

            ThreeAxis gyro_sum_;
            ThreeAxis gyro_sum_of_squares_;
            uint16_t gyro_sample_count_;
            ThreeAxis gyro_bias_;
            uint32_t gyro_variance_sample_time_msec_;
            ThreeAxisLpf accel_lpf_;
            ThreeAxisLpf gyro_lpf_;

            static auto Square(const ThreeAxis & vec) -> ThreeAxis
            {
                return vec * vec;
            }

            static auto Scale(const ThreeAxis & vec, const int16_t s) -> ThreeAxis
            {
                return vec * 2 * (float)s / 65536;
            }
    };
}
