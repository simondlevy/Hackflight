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
#include <firmware/imu/lpf.hpp>

namespace hf {

    class ThreeAxisLpf {

        public:

            IMU::ThreeAxis output;

            ThreeAxisLpf() = default;

            ThreeAxisLpf(const IMU::ThreeAxis & output,
                    const LPF &x, const LPF &y, const LPF &z)
                : output(output), _x(x), _y(y), _z(z) {}

            ThreeAxisLpf& operator=(const ThreeAxisLpf& other) = default;

            static auto apply(const ThreeAxisLpf & lpf, const IMU::ThreeAxis & in,
                    const float cutoff_freq) -> ThreeAxisLpf
            {
                const auto x = LPF::apply(lpf._x, in.x, cutoff_freq);
                const auto y = LPF::apply(lpf._y, in.y, cutoff_freq);
                const auto z = LPF::apply(lpf._z, in.z, cutoff_freq);

                const auto output = IMU::ThreeAxis(x.output, y.output, z.output);

                return ThreeAxisLpf(output, x, y, z);
            }

        private:

            LPF _x;
            LPF _y;
            LPF _z;

    }; // class ThreeAxisLpf

} // namespace hf
