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

#include <difference-allinone.hpp>

#include <control/pids/altitude.hpp>
#include <control/pids/position.hpp>
#include <control/pids/pitchroll_angle.hpp>
#include <control/pids/pitchroll_rate.hpp>
#include <control/pids/yaw_angle.hpp>
#include <control/pids/yaw_rate.hpp>
#include <vehicles/diyquad.hpp>

class SnnHelper {

    friend class ClosedLoopControl;

    private:

        static constexpr float MAX_SPIKE_TIME = 100;
        static constexpr float LANDING_ALTITUDE_METERS = 0.03;

        Difference_Helper _allinone;

        bool _hovering;

        // Hybrid SNN / PID control
        float runClimbRateController(
                const bool hovering,
                const float dt,
                const float z,
                const float dz,
                const float demand)
        {
            static const float KP = 25;
            static const float KI = 15;
            static const float ILIMIT = 5000;

            _hovering = hovering;

            static float _integral;

            const auto airborne = hovering || (z > LANDING_ALTITUDE_METERS);

            const float error = _allinone.run(demand, dz, 1);  // 1 = bias

            _integral = airborne ? 
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            const auto thrust = KP * error + KI * _integral;

            return airborne ?
                Num::fconstrain(thrust * THRUST_SCALE + THRUST_BASE,
                        THRUST_MIN, THRUST_MAX) : 0;
        }

        // Visualization helpers ----------------------------------------------

        int get_i1_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.i1.get_last_fire_time());
        }

        int get_i2_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.i2.get_last_fire_time());
        }

        int get_s_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.s.get_last_fire_time());
        }

        int get_d1_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.d1.get_last_fire_time());
        }

        int get_d2_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.d2.get_last_fire_time());
        }

        int get_s2_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.s2.get_last_fire_time());
        }

        int get_o_relative_spike_time()
        {
            return normalize_when_hovering(_allinone.proc.o.get_last_fire_time());
        }

        int normalize_when_hovering(const int get_last_fire_time)
        {
            return _hovering ?  50 * get_last_fire_time / 300 + 1: 0;
        }

        // -------------------------------------------------------------------

        void run(
                const float dt,
                const bool hovering,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                demands_t & demands)
        {
            const auto climbrate = AltitudeController::run(hovering,
                    dt, vehicleState.z, openLoopDemands.thrust);

            demands.thrust = runClimbRateController( hovering, dt,
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
};
