/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
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

#include <pids/altitude.hpp>
#include <pids/climbrate.hpp>
#include <pids/position.hpp>
#include <pids/pitchroll_angle.hpp>
#include <pids/pitchroll_rate.hpp>
#include <pids/new_yaw_angle.hpp>
#include <pids/yaw_rate.hpp>
#include <msp/serializer.hpp>
#include <teensy_pidcontrol.hpp>

namespace hf {

    class NewPidControl {

        public:

            demands_t run(
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & state,
                    const demands_t & demands_in)
            {
                demands_t demands = {};
                run(dt, hovering, state, demands_in, demands);
                return demands;
            }

            void run(
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & state,
                    const demands_t & demands_in,
                    demands_t & demands)
            {
                static float _altitude_target;

                if (_altitude_target == 0) {
                    _altitude_target = ALTITUDE_INIT_M;
                }

                _altitude_target = Num::fconstrain(
                        _altitude_target +
                        demands_in.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                const auto climbrate = AltitudeController::run(hovering,
                        dt, state.z, _altitude_target);

                demands.thrust =
                    ClimbRateController::run(
                            hovering,
                            dt,
                            state.z,
                            state.dz,
                            climbrate);

                const auto airborne = demands.thrust > 0;

                PositionController::run(
                        airborne,
                        dt,
                        state.dx, state.dy, state.psi,
                        hovering ? demands_in.pitch : 0,
                        hovering ? demands_in.roll : 0,
                        demands.roll, demands.pitch);

                runStabilizerPids(dt, state, demands_in, demands);
            }

            static void runStabilizerPids(
                    const float dt,
                    const vehicleState_t & state,
                    const demands_t & demands_in,
                    demands_t & demands_out)
            {
                const auto airborne = demands_out.thrust > 0;

                const demands_t new_demands_in ={
                    0, 0, 0, MAX_YAW_DEMAND_DPS * demands_in.yaw
                };

                PitchRollAngleController::run(
                        airborne,
                        dt,
                        state.phi, state.theta,
                        demands_out.roll, demands_out.pitch,
                        demands_out.roll, demands_out.pitch);

                PitchRollRateController::run(
                        airborne,
                        dt,
                        state.dphi, state.dtheta,
                        demands_out.roll, demands_out.pitch,
                        demands_out.roll, demands_out.pitch);

                demands_t new_demands_out = {};
                PidControl::run(dt, false, state, new_demands_in, new_demands_out);

                demands_out.yaw = new_demands_out.yaw; 
            }

            void serializeMessage(MspSerializer & serializer)
            {
                (void)serializer;
            }

            // unused; needed for sim API
            void init()
            {
            }

        private:

            static constexpr float ALTITUDE_INIT_M = 0.4;
            static constexpr float ALTITUDE_MAX_M = 1.0;
            static constexpr float ALTITUDE_MIN_M = 0.2;
            static constexpr float ALTITUDE_INC_MPS = 0.2;

            static constexpr float MAX_YAW_DEMAND_DPS = 160;     
    };
}
