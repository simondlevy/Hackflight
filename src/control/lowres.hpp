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

#include <datatypes.h>
#include <num.hpp>

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
                demands_t & demands)
        {
            (void)step;

            static float phimin, phimax;
            minmax(vehicleState.phi, phimin, phimax);
            printf("phi = %+3.3f, %+3.3f\n", phimin, phimax);

            const uint8_t dx_byte = Num::float2byte(vehicleState.dx,
                        STATE_DXY_MAX);

            const uint8_t dy_byte = Num::float2byte(vehicleState.dy,
                        STATE_DXY_MAX);

            const uint8_t z_byte = Num::float2byte(vehicleState.z,
                        STATE_Z_MIN, STATE_Z_MAX);

            const uint8_t dz_byte = Num::float2byte(vehicleState.dz,
                        STATE_DZ_MAX);

            const uint8_t psi_byte = Num::float2byte(vehicleState.psi,
                        STATE_PSI_MAX);

            const uint8_t phi_byte = Num::float2byte(vehicleState.phi,
                        STATE_PHITHETA_MAX);

            const uint8_t theta_byte = Num::float2byte(vehicleState.theta,
                        STATE_PHITHETA_MAX);

            const uint8_t dpsi_byte = Num::float2byte(vehicleState.dpsi,
                        STATE_DPSI_MAX);

            const float climbrate = AltitudeController::run(hovering, dt, z_byte,
                    openLoopDemands.thrust);

            demands.thrust = ClimbRateController::run(hovering, dt, z_byte,
                    dz_byte, climbrate);

            const auto airborne = demands.thrust > 0;

            const auto yaw = YawAngleController::run(
                    airborne, dt, psi_byte, openLoopDemands.yaw);

            demands.yaw =
                YawRateController::run(airborne, dt, dpsi_byte, yaw);

            PositionController::run(
                    airborne,
                    dt,
                    dx_byte, dy_byte, psi_byte,
                    hovering ? openLoopDemands.pitch : 0,
                    hovering ? openLoopDemands.roll : 0,
                    demands.roll, demands.pitch);

            PitchRollAngleController::run(
                    airborne, dt, phi_byte, theta_byte, demands.roll,
                    demands.pitch, demands.roll, demands.pitch);

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

        void minmax(const float val, float & min, float & max)
        {
            min = val < min ? val : min;
            max = val > max ? val : max;
        }
};
