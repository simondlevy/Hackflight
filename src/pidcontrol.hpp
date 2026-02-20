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
#include <pids/yaw_angle.hpp>
#include <pids/yaw_rate.hpp>
#include <msp/serializer.hpp>

namespace hf {

    class PidControl {

        public:

            setpoint_t run(
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & vehicleState,
                    const setpoint_t & openLoopDemands)
            {
                setpoint_t setpoint = {};
                run(dt, hovering, vehicleState, openLoopDemands, setpoint);
                return setpoint;
            }

            void run(
                    const float dt,
                    const bool hovering,
                    const vehicleState_t & vehicleState,
                    const setpoint_t & openLoopDemands,
                    setpoint_t & setpoint)
            {
                static float _altitude_target;

                if (_altitude_target == 0) {
                    _altitude_target = ALTITUDE_INIT_M;
                }

                _altitude_target = Num::fconstrain(
                        _altitude_target +
                        openLoopDemands.thrust * ALTITUDE_INC_MPS * dt,
                        ALTITUDE_MIN_M, ALTITUDE_MAX_M);

                const auto climbrate = AltitudeController::run(hovering,
                        dt, vehicleState.z, _altitude_target);

                setpoint.thrust =
                    ClimbRateController::run(
                            hovering,
                            dt,
                            vehicleState.z,
                            vehicleState.dz,
                            climbrate);

                const auto airborne = setpoint.thrust > 0;

                PositionController::run(
                        airborne,
                        dt,
                        vehicleState.dx, vehicleState.dy, vehicleState.psi,
                        hovering ? openLoopDemands.pitch : 0,
                        hovering ? openLoopDemands.roll : 0,
                        setpoint.roll, setpoint.pitch);


                runStabilizerPids(dt, dt, vehicleState, openLoopDemands, setpoint);
            }

            static void runStabilizerPids(
                    const float dt,
                    const float yaw_demand_inc,
                    const vehicleState_t & vehicleState,
                    const setpoint_t & openLoopDemands,
                    setpoint_t & setpoint)
            {
                const auto airborne = setpoint.thrust > 0;

                static float _yaw_angle_target;

                _yaw_angle_target = Num::cap_angle(_yaw_angle_target +
                        YAW_DEMAND_MAX * openLoopDemands.yaw * yaw_demand_inc);

                const auto yaw = YawAngleController::run(
                        airborne, dt, vehicleState.psi, _yaw_angle_target);

                setpoint.yaw =
                    YawRateController::run(airborne, dt, vehicleState.dpsi, yaw);

                PitchRollAngleController::run(
                        airborne,
                        dt,
                        vehicleState.phi, vehicleState.theta,
                        setpoint.roll, setpoint.pitch,
                        setpoint.roll, setpoint.pitch);

                PitchRollRateController::run(
                        airborne,
                        dt,
                        vehicleState.dphi, vehicleState.dtheta,
                        setpoint.roll, setpoint.pitch,
                        setpoint.roll, setpoint.pitch);
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

            static constexpr float YAW_DEMAND_MAX = 200;
    };
}
