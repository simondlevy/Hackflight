/**
 * Copyright 2025 Simon D. Levy
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

#include <clock.hpp>
#include <control/pids/climbrate.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>
#include <datatypes.h>
#include <num.hpp>
#include <tennlab/differencer.hpp>
#include <vehicles/diyquad.hpp>

#include <posix-utils/socket.hpp>

static float runDifferenceSnn(const float demand, const float actual)
{
    static DifferenceNetwork _network;

    static bool _initialized;

    // Initialize the first time around
    if (!_initialized) {

        // true = visualize
        _network.init(true);

        _initialized = true;
    }

    return _network.run(demand, actual);
}

static float runAltitudeController(
        const bool hovering,
        const float dt,
        const float z,
        const float thrust)
{
    static constexpr float KP = 2;
    static constexpr float KI = 0.5;
    static constexpr float ILIMIT = 5000;
    static constexpr float VEL_MAX = 1;
    static constexpr float VEL_MAX_OVERHEAD = 1.10;
    static constexpr float LANDING_SPEED_MPS = 0.15;

    static float _integral;

    const auto error = runDifferenceSnn(thrust, z);

    _integral = hovering ?
        Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

    return hovering ? 
        Num::fconstrain(KP * error + KI * _integral,
                fmaxf(VEL_MAX, 0.5f)  * VEL_MAX_OVERHEAD) :
        -LANDING_SPEED_MPS;
}



static void runClosedLoopControl(
        const float dt,
        const bool hovering,
        const vehicleState_t & vehicleState,
        const demands_t & openLoopDemands,
        const float landingAltitudeMeters,
        demands_t & demands)
{
    (void)landingAltitudeMeters;

    const auto climbrate = runAltitudeController(hovering,
            dt, vehicleState.z, openLoopDemands.thrust);

    demands.thrust =
        ClimbRateController::run(
                hovering,
                landingAltitudeMeters,
                dt,
                vehicleState.z,
                vehicleState.dz,
                climbrate);

    const auto airborne = demands.thrust > 0;

    const auto yaw = YawAngleController::run(
            airborne, dt, vehicleState.psi, openLoopDemands.yaw);

    demands.yaw =
        YawRateController::run(airborne, dt, vehicleState.dpsi, yaw);

    PositionController::run(
            airborne,
            dt,
            vehicleState.dx, vehicleState.dy, vehicleState.psi,
            hovering ? openLoopDemands.pitch : 0,
            hovering ? openLoopDemands.roll : 0,
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
