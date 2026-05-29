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

            static constexpr float TILT_ANGLE_FLIPPED_MIN_DEG = 75;

            static constexpr uint32_t FAILSAFE_MSEC = 500;

        public:

            static auto updateMode(
                    const uint32_t msecCurr,
                    const VehicleState & state,
                    const bool isGyroCalibrated,
                    const bool rxRequestedArming,
                    const uint32_t rxMsecPrev,
                    const ImuFilter & imufilt,
                    const mode_e mode) -> mode_e
            {
                const auto wantArming = 

                    // Disable arming while gyro is calibrating
                    !isGyroCalibrated ? false :

                    // Check receiver timeout
                    checkFailsafe(msecCurr, rxMsecPrev, rxRequestedArming);

                // Run a little state-transition machine to update flight mode
                return 

                    // Panic mode: can't recover
                    mode == MODE_PANIC ? MODE_PANIC :

                    //  Vehicle flipped over: enter panic mode
                    isFlipped(state) ? MODE_PANIC :

                    // Want arm and safe to arm: enter armed mode
                    wantArming && imufilt.isGyroCalibrated ? MODE_ARMED :

                    // Want disarm: enter idle mode
                    mode == MODE_ARMED && !wantArming ? MODE_IDLE :

                    //  Default: stay in current mode
                    mode;
            }

            static auto updateModeMsp(
                    const uint32_t msecCurr,
                    const VehicleState & state,
                    const bool isGyroCalibrated,
                    const msp_message_t & message,
                    const ImuFilter & imufilt,
                    const mode_e mode) -> mode_e
            {
                const auto msecPrev = message.timestamp_msec;

                // Run a little state-transition machine to update flight mode
                return  

                    // Transmission timed out; trigger failsafe
                    msecPrev > 0 && msecCurr > msecPrev &&
                    msecCurr - msecPrev > FAILSAFE_MSEC ? MODE_PANIC :

                    // Vehicle flipped over: enter panic mode
                    isFlipped(state) ? MODE_PANIC :

                    // Panic mode: can't recover
                    mode == MODE_PANIC ? MODE_PANIC :

                    // Disable arming while gyro is calibrating
                    !isGyroCalibrated ? MODE_IDLE :

                    // Want arm and safe to arm: enter armed mode
                    mode ==  MODE_IDLE && message.is_armed ? MODE_ARMED :

                    // Armed and want hover; enter hover mode
                    mode == MODE_ARMED && message.is_hovering ? MODE_HOVERING :

                    // Hovering and want landing; enter landing mode
                    mode == MODE_HOVERING && !message.is_hovering ? MODE_ARMED :

                    // Don't want arming; enter idle mode
                    !message.is_armed ? MODE_IDLE :

                    //  Default: stay in current mode
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
                return fabs(angle) > TILT_ANGLE_FLIPPED_MIN_DEG;
            }

            static auto checkFailsafe(
                    const uint32_t msec_curr,
                    const uint32_t msec_prev,
                    const bool is_armed) -> bool
            {
                const auto timed_out = 
                    msec_prev > 0 &&
                    msec_curr > msec_prev &&
                    msec_curr - msec_prev > FAILSAFE_MSEC;

                return timed_out ? false : is_armed;
            } 
    };
}
