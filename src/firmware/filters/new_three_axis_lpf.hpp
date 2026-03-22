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
#include <firmware/filters/lpf.hpp>

namespace hf {

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
                const auto x = LPF::apply(lpf._x, in[0], cutoff_freq);
                const auto y = LPF::apply(lpf._y, in[1], cutoff_freq);
                const auto z = LPF::apply(lpf._z, in[2], cutoff_freq);

                const auto output = Vec3(x.output, y.output, z.output);

                return ThreeAxisLpf(output, x, y, z);

                return ThreeAxisLpf();
            }

        private:

            LPF _x;
            LPF _y;
            LPF _z;

    }; // class ThreeAxisLpf

} // namespace hf
