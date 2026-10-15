/*
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

#include <stdio.h>

#include <hackflight.h>
#include <firmware/fc/imu/filter.hpp>
#include <firmware/fc/imu/three_axis.hpp>
#include <firmware/fc/optical_flow.hpp>
#include <firmware/timer.hpp>

namespace hf {

    class Debugger {

        private:

            static constexpr float kReportingRate = 100;

            class Helper {

                public:

                    int count;

                    bool Ready()
                    {
                        timer_ =
                            Timer::Update(timer_, kReportingRate, millis());

                        if (Timer::IsReady(timer_)) {
                            count++;
                            return true;
                        }

                        return false;
                    }

                private:

                    Timer timer_; 
            };

        public:

            static constexpr char * kModeNames[6] = {
                (char *)"idle",
                (char *)"armed",
                (char *)"hovering",
                (char *)"autonomous",
                (char *)"panic"
            };

            static void ReportForever(const char * errmsg)
            {
                while (true) {
                    Serial.println(errmsg);
                    delay(500);
                }
            }

            static void Report(const Mode mode)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    printf("%5d | mode=%s\n", helper_.count, kModeNames[mode]);
                }
            }

            void ReportFloat(const char * label, const float value)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    printf("%5d | %s=%f\n", helper_.count, label, value);
                }
            }

            void ReportBool(const char * label, const bool value)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    printf("%5d | %s=%s\n",
                            helper_.count, label, value ? "true" : "false");
                }
            }

            static void Report(const Setpoint & setpoint)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                printf("%5d | thrust=%+3.3f roll=%+3.3f pitch=%+3.3f "
                        "yaw=%+3.3f\n",
                        helper_.count, setpoint.thrust, setpoint.roll,
                        setpoint.pitch, setpoint.yaw);
                }
            }

            void ReportHover(const VehicleState & state)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    printf("%5d | dx=%+3.3f dy=%+3.3f z=%6.3f dz=%+5.3f\n",
                            helper_.count, state.dx, state.dy, state.z,
                            state.dz);
                }
            }

            static void Report(const VehicleState & state, const bool full=false)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    if (full) {
                        printf("%5d | "
                                "dx=%+03.3f dy=%+03.3f z=%6.3f dz=%+5.3f "
                                "phi=%+03.0f dphi=%+04.0f theta=%+03.0f "
                                " dtheta=%+04.0f psi=%+04.0f dpsi=%+04.0f\n",
                                helper_.count, state.dx, state.dy, state.z,
                                state.dz, state.phi, state.dphi, state.theta,
                                state.dtheta, state.psi, state.dpsi);
                    }
                    else {
                        printf("%5d | "
                                "phi=%+03.0f dphi=%+04.0f theta=%+03.0f "
                                "dtheta=%+04.0f psi=%+04.0f dpsi=%+04.0f\n",
                                helper_.count, state.phi, state.dphi,
                                state.theta, state.dtheta, state.psi,
                                state.dpsi);
                    }
                }
            }

            static void Report(const ThreeAxis & vec)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    printf("%5d | x=%+04.0f y=%+04.0f z=%+04.0f\n",
                            helper_.count, vec.x, vec.y, vec.z);
                }
            }

            static void Report(const ImuFilter::Data & imufilt)
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    const auto g = imufilt.gyro_dps;
                    const auto a = imufilt.accel_gs;

                    printf("%5d | gx=%+04.0f gy=%+04.0f gz=%+04.0f DPS | "
                            "ax=%+05.3f ay=%+05.3f az=%+05.3f Gs\n",
                            helper_.count, g.x, g.y, g.z, a.x, a.y, a.z);
                }
            }

            void ReportMotors(const float * vals,
                    const uint8_t count,
                    const char * fmt="%f")
            {
                static Helper helper_;

                if (helper_.Ready()) {

                    printf("%5d | ", helper_.count);

                    for (uint8_t k=0; k<count; ++k) {
                        printf("m%d=", k+1);
                        printf(fmt, vals[k]);
                        printf("    ");
                    }

                    printf("\n");
                }
            }

    };
}
