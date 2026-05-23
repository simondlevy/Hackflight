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

#pragma once

#include <hackflight.h>
#include <firmware/imu/filter.hpp>

namespace hf {

    class Safety {

        private:

            static constexpr float TILT_ANGLE_FLIPPED_MIN = 75;

            static constexpr uint32_t TIMEOUT_MSEC = 500;

        public:

            static auto updateMode(
                    const VehicleState & state,
                    const bool rxRequestedArming,
                    const uint32_t msecCurr,
                    const uint32_t rxMsecPrev,
                    const ImuFilter & imufilt,
                    const mode_e mode) -> mode_e
            {
                // Check receiver timeout
                const auto isArmed =
                    checkTimeout(msecCurr, rxMsecPrev, rxRequestedArming);

                return 
                    mode == MODE_PANIC ? MODE_PANIC : // can't recover from this
                    isFlipped(state) ? MODE_PANIC :
                    isArmed && imufilt.isGyroCalibrated ? MODE_ARMED :
                    mode == MODE_ARMED && !isArmed ? MODE_IDLE :
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

            static auto checkTimeout(
                    const uint32_t msec_curr,
                    const uint32_t msec_prev,
                    const bool is_armed) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > TIMEOUT_MSEC;

                return timed_out ? false : is_armed;
            } 
    };
}
