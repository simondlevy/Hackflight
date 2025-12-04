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

#include <control/pids/altitude.hpp>
#include <control/pids/climbrate.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>
#include <serializer.hpp>

class ClosedLoopControl {

    public:

        void run(
                const float dt,
                const bool hovering,
                const vehicleState_t & vehicleState,
                const demands_t & setpointDemands,
                demands_t & demands,
                void * extra)
        {
            (void)extra;

            const auto climbrate = AltitudeController::run(hovering,
                    dt, vehicleState.z, setpointDemands.thrust);

            demands.thrust = ClimbRateController::run( hovering, dt,
                    vehicleState.z, vehicleState.dz, climbrate);

            const auto airborne = demands.thrust > 0;

            const auto yaw = YawAngleController::run(
                    airborne, dt, vehicleState.psi, setpointDemands.yaw);

            demands.yaw =
                YawRateController::run(airborne, dt, vehicleState.dpsi, yaw);

            PositionController::run(
                    airborne,
                    dt,
                    0, 0, 0,
                    hovering ? setpointDemands.pitch : 0,
                    hovering ? setpointDemands.roll : 0,
                    demands.roll, demands.pitch);

            PitchRollAngleController::run(
                    airborne,
                    dt,
                    vehicleState.phi, vehicleState.theta,
                    demands.roll, demands.pitch,
                    demands.roll, demands.pitch);

            PitchRollRateController::run(
                    airborne,
                    dt,
                    vehicleState.dphi, vehicleState.dtheta,
                    demands.roll, demands.pitch,
                    demands.roll, demands.pitch);
        }

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        // unused; needed for sim API
        void init()
        {
        }
};
