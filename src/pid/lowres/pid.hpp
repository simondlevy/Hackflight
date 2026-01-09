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

            const float climbrate = AltitudeController::run(
                    controlled,
                    dt,
                    Num::byte2float(z_byte, STATE_Z_MIN, STATE_Z_MAX),
                    setpointDemands.thrust);

            demands.thrust = ClimbRateController::run(
                    controlled,
                    dt,
                    Num::byte2float(z_byte, STATE_Z_MIN, STATE_Z_MAX),
                    Num::byte2float(dz_byte, STATE_DZ_MAX),
                    climbrate);

            const auto yaw = YawAngleController::run(
                    dt,
                    Num::byte2float(psi_byte, STATE_PSI_MAX),
                    setpointDemands.yaw);

            demands.yaw =
                YawRateController::run(
                        dt,
                        Num::byte2float(dpsi_byte, STATE_DPSI_MAX),
                        yaw);

            PositionController::run(
                    dt,
                    Num::byte2float(dx_byte, STATE_DXY_MAX),
                    Num::byte2float(dy_byte, STATE_DXY_MAX),
                    Num::byte2float(psi_byte, STATE_PSI_MAX),
                    controlled ? setpointDemands.pitch : 0,
                    controlled ? setpointDemands.roll : 0,
                    demands.roll, demands.pitch);

            PitchRollAngleController::run(
                    dt,
                    Num::byte2float(phi_byte, STATE_PHITHETA_MAX),
                    Num::byte2float(theta_byte, STATE_PHITHETA_MAX),
                    demands.roll, demands.pitch, demands.roll, demands.pitch);

            PitchRollRateController::run(
                    dt,
                    Num::byte2float(dphi_byte, STATE_DPHITHETA_MAX),
                    Num::byte2float(dtheta_byte,STATE_DPHITHETA_MAX),
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
