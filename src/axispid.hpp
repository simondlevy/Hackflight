/*
   Yaw-rate PID controller for Hackflight

   Based on  https://github.com/nickrehm/dRehmFlight

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <hackflight.h>
#include <datatypes.hpp>
#include <num.hpp>

namespace hf {

    class AxisPid {

        public:

             float target;
             float actual;
             float error;
             float integral;
             float derivative;
             float output;

            AxisPid() = default;

            AxisPid(const AxisPid & other) = default;

            AxisPid& operator=(const AxisPid&) = default;

            static auto Run(
                    const AxisPid & pid,
                    const float kP,
                    const float kI,
                    const float kD,
                    const float kIntegralLimit,
                    const float dt,
                    const bool airborne,
                    const float target,
                    const float actual) -> AxisPid
            {
                const auto error = target - actual;

                const auto integral = airborne ? 
                    Num::ConstrainFloat(pid.integral + error * dt,
                            kIntegralLimit) :
                    0;

                const auto derivative = dt > 0 ? (error - pid.error) / dt : 0; 

                const auto output = kP*error + kI*integral + kD*derivative; 

                return AxisPid(
                        target, actual, error, integral, derivative, output);
            }

             static auto Run(
                    const AxisPid & pid,
                    const float kP,
                    const float kI,
                    const float kD,
                    const float kIntegralLimit,
                     const float dt,
                    const bool airborne,
                    const float target,
                    const float actual,
                    const float derivative) -> AxisPid
            {
                const auto error = target - actual;

                const auto integral = airborne ? 
                    Num::ConstrainFloat(
                            pid.integral + error * dt, kIntegralLimit) :
                    0;

                const auto output = kP * error + kI * integral + kD * derivative; 

                return AxisPid(
                        target, actual, error, integral, derivative, output);
            }

            static auto Run(
                    const AxisPid & pid,
                    const float kP,
                    const float kI,
                    const float kIntegralLimit,
                    const bool airborne,
                    const float dt,
                    const float target,
                    const float actual,
                    const float minval,
                    const float maxval,
                    const float offset) -> AxisPid
            {
                const auto error = target - actual;

                const auto integral = airborne ? 
                    Num::ConstrainFloat(pid.integral + error * dt,
                            kIntegralLimit) :
                    0;

                const auto tmp = kP * error + kI * integral;

                const auto output = airborne ?
                    Num::ConstrainFloat(offset + tmp, minval, maxval) :
                    0;

                return AxisPid(
                        target,
                        actual,
                        error,
                        integral,
                        0,
                        output);
            }

        private:

            AxisPid(
                    const float target,
                    const float actual,
                    const float error,
                    const float integral,
                    const float derivative, 
                    const float output)
                : 
                    target(target),
                    actual(actual),
                    error(error),
                    integral(integral),
                    derivative(derivative),
                    output(output) {}
    };

}
