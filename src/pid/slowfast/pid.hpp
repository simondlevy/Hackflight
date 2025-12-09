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
                const bool controlled,
                const vehicleState_t & vehicleState,
                const demands_t & demandsIn,
                demands_t & demandsOut)
        {
                (void)dt;
                (void) controlled;
                (void)vehicleState;

                memcpy(&demandsOut, &demandsIn, sizeof(demands_t));
        }

        void runFast(
                const float dt,
                const bool controlled,
                const vehicleState_t & vehicleState,
                const demands_t & demandsIn,
                demands_t & demandsOut)
        {
            demandsOut.thrust = AltitudeController::run(controlled,
                    dt, vehicleState.z, demandsIn.thrust);

            demandsOut.yaw = YawAngleController::run(
                    dt, vehicleState.psi, demandsIn.yaw);

            PositionController::run(
                    dt,
                    vehicleState.dx, vehicleState.dy, vehicleState.psi,
                    controlled ? demandsIn.pitch : 0,
                    controlled ? demandsIn.roll : 0,
                    demandsOut.roll, demandsOut.pitch);

            PitchRollAngleController::run(
                    dt,
                    vehicleState.phi, vehicleState.theta,
                    demandsOut.roll, demandsOut.pitch,
                    demandsOut.roll, demandsOut.pitch);

            // ---------------------------------------------------------------

            demandsOut.thrust = ClimbRateController::run(controlled, dt,
                    vehicleState.z, vehicleState.dz, demandsOut.thrust);

            PitchRollRateController::run(
                    dt,
                    vehicleState.dphi, vehicleState.dtheta,
                    demandsOut.roll, demandsOut.pitch,
                    demandsOut.roll, demandsOut.pitch);

            demandsOut.yaw =
                YawRateController::run(dt, vehicleState.dpsi, demandsOut.yaw);
        }

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        void init()
        {
        }
};
