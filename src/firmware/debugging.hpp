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

#include <hackflight.h>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/imu/three_axis.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/timer.hpp>

namespace hf {

    class Debugger {

        private:

            static constexpr float kReportingRate = 100;

            class Helper {

                private:

                    Timer timer_ = Timer(kReportingRate);

                public:

                    uint32_t count;

                    bool Ready()
                    {
                        if (timer_.Ready()) {
                            count++;
                            return true;
                        }

                        return false;
                    }
            };

        public:

            static constexpr char * MODENAMES[6] = {
                (char *)"IDLE",
                (char *)"ARMED",
                (char *)"HOVERING",
                (char *)"AUTONOMOUS",
                (char *)"PANIC"
            };

            static void ReportForever(const char * errmsg)
            {
                while (true) {
                    printf("%s\n", errmsg);
                    delay(500);
                }
            }

            void Report(const Mode mode)
            {
                if (helper_.Ready()) {

                    printf("%5lu | mode=%s\n", helper_.count, MODENAMES[mode]);
                }
            }

            void ReportFloat(const char * label, const float value)
            {
                if (helper_.Ready()) {

                    printf("%5lu | %s=%f\n", helper_.count, label, value);
                }
            }

            void ReportBool(const char * label, const bool flag)
            {
                if (helper_.Ready()) {

                    printf("%5lu | %s=%s\n",
                            helper_.count, label, flag ? "true" : "false");
                }
            }

            void Report(const Setpoint & setpoint)
            {
                if (helper_.Ready()) {

                    printf("%5lu | thrust=%+3.3f roll=%+3.3f pitch=%+3.3f "
                            "yaw=%+3.3f\n",
                            helper_.count, setpoint.thrust, setpoint.roll,
                            setpoint.pitch, setpoint.yaw); }
            }

            void ReportHover(const VehicleState & state)
            {
                if (helper_.Ready()) {

                    printf("%5lu | dx=%+3.3f dy=%+3.3f z=%6.3f dz=%+5.3f\n",
                            helper_.count, state.dx, state.dy, state.z,
                            state.dz);
                }
            }

            void Report(const VehicleState & state, const bool full=false)
            {
                if (helper_.Ready()) {

                    if (full) {
                        printf("%5lu | "
                                "dx=%+03.3f dy=%+03.3f z=%6.3f dz=%+5.3f "
                                "phi=%+03.0f dphi=%+04.0f theta=%+03.0f "
                                " dtheta=%+04.0f psi=%+04.0f dpsi=%+04.0f\n",
                                helper_.count, state.dx, state.dy, state.z,
                                state.dz, state.phi, state.dphi, state.theta,
                                state.dtheta, state.psi, state.dpsi);
                    }
                    else {
                        printf("%5lu | "
                                "phi=%+03.0f dphi=%+04.0f theta=%+03.0f "
                                "dtheta=%+04.0f psi=%+04.0f dpsi=%+04.0f\n",
                                helper_.count, state.phi, state.dphi,
                                state.theta, state.dtheta, state.psi,
                                state.dpsi);
                    }
                }
            }

            void Report(const IMU::ThreeAxisRaw & raw)
            {
                if (helper_.Ready()) {

                    printf("%5lu | x=%+05d y=%+05d z=%+05d\n",
                            helper_.count, raw.x, raw.y, raw.z);
                }
            }

            void Report(const ThreeAxis & vec)
            {
                if (helper_.Ready()) {

                    printf("%5lu | x=%+04.0f y=%+04.0f z=%+04.0f\n",
                            helper_.count, vec.x, vec.y, vec.z);
                }
            }

            void Report(const ImuFilter::Data & imufilt)
            {
                if (helper_.Ready()) {

                    const auto g = imufilt.gyro_dps;
                    const auto a = imufilt.accel_gs;

                    printf("%5lu | gx=%+04.0f gy=%+04.0f gz=%+04.0f DPS | "
                            "ax=%+05.3f ay=%+05.3f az=%+05.3f Gs\n",
                            helper_.count, g.x, g.y, g.z, a.x, a.y, a.z);
                }
            }

            void Report(const IMU::RawData & imuraw)
            {
                if (helper_.Ready()) {

                    const auto gyro = imuraw.gyro;
                    const auto accel = imuraw.accel;

                    printf("%5lu | gx=%+05d gy=%+05d gz=%+05d | "
                            "ax=%+05d ay=%+05d az=%+05d\n",
                            helper_.count, gyro.x, gyro.y, gyro.z,
                            accel.x, accel.y, accel.z);
                }
            }

            void ReportMotors(
                    const float * vals,
                    const uint8_t count,
                    const char * fmt="%f")
            {
                if (helper_.Ready()) {

                    printf("%5lu | ", helper_.count);

                    for (uint8_t k=0; k<count; ++k) {
                        printf("m%d=", k+1);
                        printf(fmt, vals[k]);
                        printf("    ");
                    }

                    printf("\n");
                }
             }

        private:

            Helper helper_;


    };
}
