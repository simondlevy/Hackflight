/*
   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <firmware/datatypes.hpp>

namespace hf {

    class ThreeAxisFilter {

        public:

            ThreeAxisFilter() : _prev(Vec3(0, 0, 0)) {}

            ThreeAxisFilter& operator=(const ThreeAxisFilter& other) = default;

            Vec3 run(
                    const Vec3 & raw,
                    const Vec3 & error,
                    const float scale,
                    const float coeff)
            {
                const auto curr = raw / scale - error;

                const auto output = _prev * (1 - coeff) + curr * coeff;

                _prev = curr;

                return output;
            }

        private:

            Vec3 _prev;
    };
}
