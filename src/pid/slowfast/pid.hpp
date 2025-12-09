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

#include <pid/pids/altitude.hpp>
#include <pid/pids/climbrate.hpp>
#include <pid/pids/position.hpp>
#include <pid/pids/pitchroll_angle.hpp>
#include <pid/pids/pitchroll_rate.hpp>
#include <pid/pids/yaw_angle.hpp>
#include <pid/pids/yaw_rate.hpp>
#include <serializer.hpp>

class PidControl {

    public:

        void runSlow(
                const float dt,
                const flightMode_t flightMode,
                const vehicleState_t & vehicleState,
                const demands_t & setpointDemands,
                demands_t & demands)
        {
                (void)dt;
                (void) flightMode;
                (void)vehicleState;
                (void)setpointDemands;
                (void)demands;
        }

        void runFast(
                const float dt,
                const flightMode_t flightMode,
                const vehicleState_t & vehicleState,
                const demands_t & setpointDemands,
                demands_t & demands)
        {
            const bool controlled = flightMode == MODE_HOVERING ||
                flightMode == MODE_AUTONOMOUS;

            // ---------------------------------------------------------------

            demands.thrust = AltitudeController::run(controlled,
                    dt, vehicleState.z, setpointDemands.thrust);

            const auto yaw = YawAngleController::run(
                    dt, vehicleState.psi, setpointDemands.yaw);

            PositionController::run(
                    dt,
                    vehicleState.dx, vehicleState.dy, vehicleState.psi,
                    controlled ? setpointDemands.pitch : 0,
                    controlled ? setpointDemands.roll : 0,
                    demands.roll, demands.pitch);

            PitchRollAngleController::run(
                    dt,
                    vehicleState.phi, vehicleState.theta,
                    demands.roll, demands.pitch,
                    demands.roll, demands.pitch);

            // ---------------------------------------------------------------

            demands.thrust = ClimbRateController::run(controlled, dt,
                    vehicleState.z, vehicleState.dz, demands.thrust);

            PitchRollRateController::run(
                    dt,
                    vehicleState.dphi, vehicleState.dtheta,
                    demands.roll, demands.pitch,
                    demands.roll, demands.pitch);
            demands.yaw =
                YawRateController::run(dt, vehicleState.dpsi, yaw);

        }

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        void init()
        {
        }
};
