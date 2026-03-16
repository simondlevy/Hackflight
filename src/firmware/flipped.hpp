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

namespace hf {

    class FlippedCheck {

        private:

            static constexpr float TILT_ANGLE_FLIPPED_MIN = 75;

        public:

            static auto isFlipped(const VehicleState & state) -> bool
            {
                return isFlippedAngle(state.theta) ||
                    isFlippedAngle(state.phi); 
            }

        private:

            static auto isFlippedAngle(const float angle) -> bool
            {
                return fabs(angle) > TILT_ANGLE_FLIPPED_MIN;
            }
    };
}
