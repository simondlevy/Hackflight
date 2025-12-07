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

#include <pid.hpp>
#include <pid/pids/altitude.hpp>
#include <pid/pids/climbrate.hpp>
#include <pid/pids/position.hpp>
#include <pid/pids/pitchroll_angle.hpp>
#include <pid/pids/pitchroll_rate.hpp>
#include <pid/pids/yaw_angle.hpp>
#include <pid/pids/yaw_rate.hpp>

void PidControl::run(
        const float dt,
        const flightMode_t flightMode,
        const vehicleState_t & vehicleState,
        const demands_t & setpointDemands,
        demands_t & demands)
{
    const bool airborne = flightMode == MODE_HOVERING ||
        flightMode == MODE_AUTONOMOUS;

    const auto climbrate = AltitudeController::run(airborne,
            dt, vehicleState.z, setpointDemands.thrust);

    demands.thrust = ClimbRateController::run(airborne, dt,
            vehicleState.z, vehicleState.dz, climbrate);

    const auto posthrust = demands.thrust > 0;

    const auto yaw = YawAngleController::run(
            posthrust, dt, vehicleState.psi, setpointDemands.yaw);

    demands.yaw =
        YawRateController::run(posthrust, dt, vehicleState.dpsi, yaw);

    PositionController::run(
            posthrust,
            dt,
            vehicleState.dx, vehicleState.dy, vehicleState.psi,
            airborne ? setpointDemands.pitch : 0,
            airborne ? setpointDemands.roll : 0,
            demands.roll, demands.pitch);

    PitchRollAngleController::run(
            posthrust,
            dt,
            vehicleState.phi, vehicleState.theta,
            demands.roll, demands.pitch,
            demands.roll, demands.pitch);

    PitchRollRateController::run(
            posthrust,
            dt,
            vehicleState.dphi, vehicleState.dtheta,
            demands.roll, demands.pitch,
            demands.roll, demands.pitch);
}

void PidControl::serializeMessage(MspSerializer & serializer)
{
    (void)serializer;
}

void PidControl::init()
{
}
