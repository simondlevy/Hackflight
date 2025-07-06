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

#include <difference_network.hpp>

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

        DifferenceNetwork net;

        // Hybrid SNN / PID control
        float runClimbRateController(
                const bool hovering,
                const float z0,
                const float dt,
                const float z,
                const float dz,
                const float demand)
        {
            static const float KP = 25;
            static const float KI = 15;
            static const float ILIMIT = 5000;

            static float _integral;

            const auto airborne = hovering || (z > z0);

            const int timesteps = 3 * MAX_SPIKE_TIME + 2;

            // Note clamped value for third input
            net.run(timesteps,
                    value_to_spike_time(demand),
                    value_to_spike_time(dz),
                    value_to_spike_time(1));

            // Handle edge case
            const int output_spike_time = net.get_o_spike_time();
            const int time = output_spike_time == MAX_SPIKE_TIME + 1 ? -2 :
                output_spike_time;

            // Convert the firing time to a difference in [-2,+2]
            const float error =
                ((float)time-MAX_SPIKE_TIME)*2 / MAX_SPIKE_TIME - 2;
 
            _integral = airborne ? 
                Num::fconstrain(_integral + error * dt, ILIMIT) : 0;

            const auto thrust = KP * error + KI * _integral;

            return airborne ?
                Num::fconstrain(thrust * THRUST_SCALE + THRUST_BASE,
                        THRUST_MIN, THRUST_MAX) : 0;
        }

        // Encoder -----------------------------------------------------------

        int value_to_spike_time(const float value)
        {
            return (int)(round(MAX_SPIKE_TIME * (1 - value) / 2));
        }

        // Decoder ------------------------------------------------------------

        int get_i1_spike_count()
        {
            return spike_time_to_spike_count(net.get_i1_spike_time(), 50, 50);
        }

        int get_i2_spike_count()
        {
            return spike_time_to_spike_count(net.get_i2_spike_time(), 50, 50);
        }

        int get_s_spike_count()
        {
            return 1;
        }

        int get_d1_spike_count()
        {
            return spike_time_to_spike_count(net.get_d1_spike_time(), 25, 25);
        }

        int get_d2_spike_count()
        {
            return spike_time_to_spike_count(net.get_d2_spike_time(), 25, 25);
        }

        int get_s2_spike_count()
        {
            return spike_time_to_spike_count(net.get_s2_spike_time(), -5, 40);
        }

        int get_o_spike_count()
        {
            return spike_time_to_spike_count(net.get_o_spike_time(), -5, 40);
        }

        // Utility -----------------------------------------------------------

        int spike_time_to_spike_count(
                const int spike_time, const int scale, const int offset)
        {
            const float value = -(2.f * spike_time / MAX_SPIKE_TIME - 1);
            return (int)(value * scale + offset);
        }

        // -------------------------------------------------------------------

        void run(
                const float dt,
                const bool hovering,
                const vehicleState_t & vehicleState,
                const demands_t & openLoopDemands,
                const float landingAltitudeMeters,
                demands_t & demands)
        {
            const auto climbrate = AltitudeController::run(hovering,
                    dt, vehicleState.z, openLoopDemands.thrust);

            demands.thrust =
                runClimbRateController(
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

};
