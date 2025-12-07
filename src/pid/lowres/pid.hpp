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

#include <pid/lowres/altitude.hpp>
#include <pid/lowres/climbrate.hpp>
#include <pid/lowres/position.hpp>
#include <pid/lowres/pitchroll_angle.hpp>
#include <pid/lowres/pitchroll_rate.hpp>
#include <pid/lowres/yaw_angle.hpp>
#include <pid/lowres/yaw_rate.hpp>
#include <serializer.hpp>


class PidControl {

    public:

        void run(
                const float dt,
                const flightMode_t flightMode,
                const vehicleState_t & vehicleState,
                const demands_t & setpointDemands,
                demands_t & demands)
        {
            const bool airborne = flightMode == MODE_HOVERING ||
                flightMode == MODE_AUTONOMOUS;

            const uint8_t dx_byte = Num::float2byte(vehicleState.dx,
                    STATE_DXY_MAX);

            const uint8_t dy_byte = Num::float2byte(vehicleState.dy,
                    STATE_DXY_MAX);

            const uint8_t z_byte = Num::float2byte(vehicleState.z,
                    STATE_Z_MIN, STATE_Z_MAX);

            const uint8_t dz_byte = Num::float2byte(vehicleState.dz,
                    STATE_DZ_MAX);

            const uint8_t phi_byte = Num::float2byte(vehicleState.phi,
                    STATE_PHITHETA_MAX);

            const uint8_t dphi_byte = Num::float2byte(vehicleState.dphi,
                    STATE_DPHITHETA_MAX);

            const uint8_t theta_byte = Num::float2byte(vehicleState.theta,
                    STATE_PHITHETA_MAX);

            const uint8_t dtheta_byte = Num::float2byte(vehicleState.dtheta,
                    STATE_DPHITHETA_MAX);

            const uint8_t psi_byte = Num::float2byte(vehicleState.psi,
                    STATE_PSI_MAX);

            const uint8_t dpsi_byte = Num::float2byte(vehicleState.dpsi,
                    STATE_DPSI_MAX);

            const float climbrate = AltitudeController::run(airborne, dt, z_byte,
                    setpointDemands.thrust);

            demands.thrust = ClimbRateController::run(airborne, dt, z_byte,
                    dz_byte, climbrate);

            const auto posthrust = demands.thrust > 0;

            const auto yaw = YawAngleController::run(
                    posthrust, dt, psi_byte, setpointDemands.yaw);

            demands.yaw =
                YawRateController::run(posthrust, dt, dpsi_byte, yaw);

            PositionController::run(
                    posthrust,
                    dt,
                    dx_byte, dy_byte, psi_byte,
                    airborne ? setpointDemands.pitch : 0,
                    airborne ? setpointDemands.roll : 0,
                    demands.roll, demands.pitch);

            PitchRollAngleController::run(
                    posthrust, dt, phi_byte, theta_byte, demands.roll,
                    demands.pitch, demands.roll, demands.pitch);

            PitchRollRateController::run( posthrust, dt, dphi_byte, dtheta_byte,
                    demands.roll, demands.pitch, demands.roll, demands.pitch);

            demands.thrust = quantize(demands.thrust, 0, UINT16_MAX);
            demands.roll = quantize(demands.roll);
            demands.pitch = quantize(demands.pitch);
            demands.yaw = quantize(demands.yaw, 32767);
        }

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        void init()
        {
        }

    private:

        static float quantize(const float val, const float min, const float max)
        {
            return Num::byte2float(Num::float2byte(val, min, max), min, max);
        }

        static float quantize(const float val, const float max)
        {
            return Num::byte2float(Num::float2byte(val, max), max);
        }

        static float quantize(const float val)
        {
            return quantize(val, 180000);
        }

};
