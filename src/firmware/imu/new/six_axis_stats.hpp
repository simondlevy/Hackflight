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

namespace hf {

    class SixAxisStats {

        public:

            Vec3 mean;
            Vec3 variance;

            SixAxisStats() = default;

            SixAxisStats
                (const Vec3& mean, const Vec3& variance) 
                : mean(mean), variance(variance) {}

            SixAxisStats& operator=(const SixAxisStats& other) = default;

     }; // class SixAxisStats

} // namespace hf
