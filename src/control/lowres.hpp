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

#include <control/lowres/altitude.hpp>
#include <control/lowres/climbrate.hpp>
#include <control/lowres/position.hpp>
#include <control/lowres/pitchroll_angle.hpp>
#include <control/lowres/pitchroll_rate.hpp>
#include <control/lowres/yaw_angle.hpp>
#include <control/lowres/yaw_rate.hpp>

#include <msp/serializer.hpp>

class ClosedLoopControl {

    public:

        void run(
                const uint32_t step,
                const float dt,
                const bool hovering,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                const float landingAltitudeMeters,
                demands_t & demands)
        {
            (void)step;

            const uint8_t z = float2byte(vehicleState.z, ZMIN, ZMAX);
            const uint8_t thrust = float2byte(openLoopDemands.thrust, THRUSTMIN, THRUSTMAX);

            printf("z=(%3.3f,%03d) thrust=(%3.3f,%03d)\n", 
                    vehicleState.z, z,
                    openLoopDemands.thrust, thrust);
     
            const auto climbrate = AltitudeController::run(
                    hovering,
                    dt,
                    vehicleState.z,
                    openLoopDemands.thrust);

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

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        // unused; needed for sim API
        void init()
        {
        }

    private:

        static constexpr float ZMIN = 0;
        static constexpr float ZMAX = 3;

        static constexpr float THRUSTMIN = 0;
        static constexpr float THRUSTMAX = 1;

        static uint8_t float2byte(const float val, const float min, const float max)
        {
            return (uint8_t)(255 * (val - min) / (max - min));
        }
};
