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

#include <control/newpids/altitude.hpp>
#include <control/newpids/yaw_angle.hpp>
#include <serializer.hpp>

#include <control/pids/climbrate.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/position.hpp>
#include <control/pids/yaw_rate.hpp>

class PidControl {

    public:

         void run(
                const float dt,
                const bool controlled,
                const vehicleState_t & vehicleState,
                const demands_t & demandsIn,
                demands_t & demandsOut)
        {
            const auto climbrate = AltitudeController::run(controlled,
                    dt, vehicleState.z, demandsIn.thrust);

            const auto yaw = YawAngleController::run(
                    dt, vehicleState.psi, demandsIn.yaw);

            PositionController::run(controlled, dt, vehicleState.dx,
                    vehicleState.dy, vehicleState.psi, controlled ?
                    demandsIn.pitch : 0, controlled ? demandsIn.roll : 0,
                    demandsOut.roll, demandsOut.pitch);

            PitchRollAngleController::run(controlled, dt, vehicleState.phi,
                    vehicleState.theta, demandsOut.roll, demandsOut.pitch,
                    demandsOut.roll, demandsOut.pitch);

            demandsOut.thrust = ClimbRateController::run(controlled,
                    LANDING_ALTITUDE_METERS, dt, vehicleState.z,
                    vehicleState.dz, climbrate);

            PitchRollRateController::run(controlled, dt, vehicleState.dphi,
                    vehicleState.dtheta, demandsOut.roll, demandsOut.pitch,
                    demandsOut.roll, demandsOut.pitch);

            demandsOut.yaw =
                YawRateController::run(true, dt, vehicleState.dpsi, yaw);

        }

        void serializeMessage(MspSerializer & serializer)
        {
            (void)serializer;
        }

        void init()
        {
        }

    private:

        static constexpr float LANDING_ALTITUDE_METERS = 0.03;

};
