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

    class PidControl {

        public:

            PidControl() = default;

            PidControl& operator=(const PidControl& other) = default;

            Setpoint setpoint;

            static void run(
                    PidControl & self,
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & state,
                    const Setpoint & setpoint_in)
            {
                // Altitude hold ---------------------------------------------

                if (self._altitude_target == 0) {
                    self._altitude_target = ALTITUDE_INIT_M;
                }

                self._altitude_target = Num::fconstrain(
                        self._altitude_target +
                        setpoint_in.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                self._altitude_pid = AltitudeController::run(self._altitude_pid,
                        hovering, dt, self._altitude_target, state.z);

                self._climbrate_pid = ClimbRateController::run(self._climbrate_pid,
                        hovering, dt, self._altitude_pid.output, state.z, state.dz);

                const auto thrust = self._climbrate_pid.output;

                self.setpoint.thrust = thrust;

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

                self._position_y_pid = PositionController::run(self._position_y_pid,
                        airborne, dt, setpoint_in.roll, dyb);

                self._position_x_pid = PositionController::run(self._position_x_pid,
                        airborne, dt, setpoint_in.pitch, dxb);

                //  Stabilization ---------------------------------------------

                self._roll_pid = RollPitchPid::run(self._roll_pid, dt, airborne,
                        self._position_y_pid.output, state.phi, state.dphi);

                self.setpoint.roll = self._roll_pid.output;

                self._pitch_pid = RollPitchPid::run(self._pitch_pid, dt, airborne,
                        self._position_x_pid.output, state.theta, state.dtheta);

                self.setpoint.pitch = self._pitch_pid.output;

                self._yaw_pid = YawPid::run(self._yaw_pid, dt, airborne, 
                        setpoint_in.yaw * MAX_YAW_DEMAND_DPS, state.dpsi);

                self.setpoint.yaw = self._yaw_pid.output;
            }

            void runStabilizer(
                    const float dt,
                    const bool airborne,
                    const float roll_angle_demand,
                    const float pitch_angle_demand,
                    const float yaw_demand,
                    const vehicleState_t & state,
                    Setpoint & setpoint_out)
            {
                setpoint_out.roll = _roll_pid.run(
                        dt, airborne, roll_angle_demand, state.phi, state.dphi);

                setpoint_out.pitch = _pitch_pid.run(
                        dt, airborne, pitch_angle_demand, state.theta, state.dtheta);

                setpoint_out.yaw = _yaw_pid.run(dt, airborne, 
                        yaw_demand * MAX_YAW_DEMAND_DPS, state.dpsi);
            }

            void serializeMessage(MspSerializer & serializer)
            {
                (void)serializer;
            }

        private:

            static constexpr float ALTITUDE_INIT_M = 0.4;
            static constexpr float ALTITUDE_MAX_M = 1.0;
            static constexpr float ALTITUDE_MIN_M = 0.2;
            static constexpr float ALTITUDE_INC_MPS = 0.2;

            static constexpr float MAX_YAW_DEMAND_DPS = 160;     

            float _altitude_target;

            AltitudeController _altitude_pid;

            ClimbRateController _climbrate_pid;

            PositionController _position_x_pid;
            PositionController _position_y_pid;

            RollPitchPid _pitch_pid;
            RollPitchPid _roll_pid;

            YawPid _yaw_pid;
    };
}
