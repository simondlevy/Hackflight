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

#include <newpids/altitude.hpp>
#include <newpids/climbrate.hpp>
#include <newpids/position.hpp>
#include <newpids/rollpitch.hpp>
#include <newpids/yaw.hpp>
#include <msp/serializer.hpp>

namespace hf {

    class NewPidControl {

        public:

            demands_t run(
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & state,
                    const demands_t & demands_in)
            {
                demands_t demands_out = {};
                run(dt, hovering, state, demands_in, demands_out);
                return demands_out;
            }

            void run(
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & state,
                    const demands_t & demands_in,
                    demands_t & demands_out)
            {
                // Altitude hold ---------------------------------------------

                static float _altitude_target;

                if (_altitude_target == 0) {
                    _altitude_target = ALTITUDE_INIT_M;
                }

                _altitude_target = Num::fconstrain(
                        _altitude_target +
                        demands_in.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                const auto climbrate = AltitudeController::run(hovering,
                        dt, _altitude_target, state.z);

                const auto thrust = ClimbRateController::run(
                    hovering, dt, climbrate, state.z, state.dz);

                demands_out.thrust = thrust;

                // Position hold ---------------------------------------------

                const auto airborne = thrust > 0;

                // Rotate world-coordinate velocities into body coordinates
                const auto dxw = state.dx;
                const auto dyw = state.dy;
                const auto psi = Num::DEG2RAD * state.psi;
                const auto cospsi = cos(psi);
                const auto sinpsi = sin(psi);
                const auto dxb =  dxw * cospsi + dyw * sinpsi;
                const auto dyb = -dxw * sinpsi + dyw * cospsi;       

                const auto roll_angle_demand =_position_y_pid.run(
                        airborne, dt, demands_in.roll, dyb);

                const auto pitch_angle_demand = _position_x_pid.run(
                        airborne, dt, demands_in.pitch, dxb);

                //  Stabilization ---------------------------------------------

                runStabilizer(dt, airborne,
                        roll_angle_demand, pitch_angle_demand, demands_in.yaw,
                        state, demands_out);
            }

            void runStabilizer(
                    const float dt,
                    const bool airborne,
                    const float roll_angle_demand,
                    const float pitch_angle_demand,
                    const float yaw_demand,
                    const vehicleState_t & state,
                    demands_t & demands_out)
            {
                demands_out.roll = _roll_pid.run(
                        dt, airborne, roll_angle_demand, state.phi, state.dphi);

                demands_out.pitch = _pitch_pid.run(
                        dt, airborne, pitch_angle_demand, state.theta, state.dtheta);

                demands_out.yaw = _yaw_pid.run(dt, airborne, 
                        yaw_demand * MAX_YAW_DEMAND_DPS, state.dpsi);
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

            PositionController _position_x_pid;
            PositionController _position_y_pid;

            RollPitchPid _pitch_pid;
            RollPitchPid _roll_pid;

            YawPid _yaw_pid;
    };
}
