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

#include <control/pids/altitude.hpp>
#include <control/pids/climbrate.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_rate.hpp>
#include <serializer.hpp>
#include <num.hpp>

class YawAngleController {

    public:

        /**
          * Input is desired angle (deg) estimated actual angle (deg) from EKF;
            ouputput is angles-per-second demand sent to YawRateController.
          */
        static float run(
                const bool airborne,  // ignore this
                const float dt,       // can be a constant if needed
                const float psi,      // estimated angle
                const float yaw)      // desired angle
        {
            printf("dt=%f\n", dt);

            static float _target;

            _target = airborne ?
                cap(_target + DEMAND_MAX * yaw * dt) : psi;

            return airborne ?  (float)plankpid(_target, psi, dt) : 0;
        }

    private:

        static constexpr float KP = 6;
        static constexpr float KI = 1;
        static constexpr float KD = 0.35;
        static constexpr float ILIMIT = 360;

        static constexpr float DEMAND_MAX = 200;

        static double plankpid(const float target, const float actual, const float dt)
        {
            static long long i_m;
            static long long p_m;

            const long long dtinv = (long long) (1/dt);
            const long long d_num = (long long) (KD * (double) dtinv);
            const long long multiplier = dtinv * dtinv;

            const long long t_m = (long long) (target * multiplier);
            const long long a_m = (long long) (actual * multiplier);

            const long long e_m = t_m - a_m;            // E_M = error * multiplier

            i_m += (e_m / (long long) dtinv) ;

            if (i_m > ILIMIT*multiplier) i_m = ILIMIT*multiplier;
            if (i_m < -ILIMIT*multiplier) i_m = -ILIMIT*multiplier;

            const long long d_m = ( e_m - p_m ) * (long long) dtinv;

            p_m = e_m;

            const long long c_m =
                ((long long) KP)*e_m + ((long long) KI) * i_m + ( d_num * d_m ) / (long long) dtinv;

            return c_m/(double)multiplier;
        }

        static float cap(float angle)
        {
            float result = angle;

            while (result > 180.0f) {
                result -= 360.0f;
            }

            while (result < -180.0f) {
                result += 360.0f;
            }

            return result;
        }
};

class ClosedLoopControl {

    public:

        void run(
                const float dt,
                const flightMode_t flightMode,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                demands_t & demands)
        {
            const bool hovering = flightMode == MODE_HOVERING;

            const auto climbrate = AltitudeController::run(hovering,
                    dt, vehicleState.z, openLoopDemands.thrust);

            demands.thrust =
                ClimbRateController::run(hovering, dt,
                        vehicleState.z, vehicleState.dz, climbrate);

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
};

static ClosedLoopControl _closedLoopControl;

