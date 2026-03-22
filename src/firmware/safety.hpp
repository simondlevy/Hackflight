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

    class Safety {

        private:

            static constexpr float TILT_ANGLE_FLIPPED_MIN = 75;

        public:

            static auto updateMode(
                    const VehicleState & state,
                    const RxData & rxdata,
                    const ImuFilter & imufilt,
                    const mode_e mode) -> mode_e
            {
                return 
                    mode == MODE_PANIC ? MODE_PANIC : // can't recover from this
                    isFlipped(state) ? MODE_PANIC :
                    rxdata.is_armed && imufilt.isGyroCalibrated ? MODE_ARMED :
                    mode == MODE_ARMED && !rxdata.is_armed ? MODE_IDLE :
                    mode;
            }

        private:

            static auto isFlipped(const VehicleState & state) -> bool
            {
                return isFlippedAngle(state.theta) ||
                    isFlippedAngle(state.phi); 
            }

            static auto isFlippedAngle(const float angle) -> bool
            {
                return fabs(angle) > TILT_ANGLE_FLIPPED_MIN;
            }
    };
}
