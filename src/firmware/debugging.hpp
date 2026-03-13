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

#include <Arduino.h>

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/timer.hpp>

namespace hf {

    class Debugger {

        private:

            static constexpr float FREQ = 100;

            class Helper {

                private:

                    Timer _timer;

                public:

                    uint32_t count;

                    bool ready()
                    {
                        if (_timer.ready(FREQ)) {
                            count++;
                            return true;
                        }

                        return false;
                    }
            };

        public:

            static void reportForever(const char * message)
            {
                while (true) {
                    printf("%s\n", message);
                    delay(500);
                }
            }

            static void report(const char * label, const bool flag)
            {
                static Helper _helper;

                if (_helper.ready()) {

                    printf("%5lu | %s=%s\n",
                            _helper.count, label, flag ? "true" : "false");
                }
            }

            static void report(const RxData & rxdata)
            {
                static Helper _helper;

                if (_helper.ready()) {

                    const auto ax = rxdata.axes;

                    printf("%5lu | armed=%d | throt_down=%d | "
                            "throt=%+3.3f roll=%+3.3f pitch=%3.3f "
                            "yaw=%+3.3f\n",
                            _helper.count, rxdata.is_armed,
                            rxdata.is_throttle_down, 
                            ax.thrust, ax.roll, ax.pitch, ax.yaw);
                }
            }

            static void report(const Setpoint & setpoint)
            {
                static Helper _helper;

                if (_helper.ready()) {

                    printf("%5lu | thrust=%+3.3f roll=%+3.3f pitch=%3.3f yaw=%+3.3f\n",
                            _helper.count, setpoint.thrust, setpoint.roll,
                            setpoint.pitch, setpoint.yaw); }
            }

            static void report(const VehicleState & state)
            {
                static Helper _helper;

                if (_helper.ready()) {

                    printf("%5lu | "
                            //"dx=%+8.3f dy=%+8.3f z=%8.3f dz=%+8.3f "
                            "phi=%+08.3f dphi=%+08.3f theta=%+08.3f dtheta=%+08.3f "
                            "psi=%+08.3f dpsi=%+08.3f\n",
                            _helper.count, 
                            //state.dx, state.dy, state.z, state.dz,
                            state.phi, state.dphi, state.theta, state.dtheta,
                            state.psi, state.dpsi);
                }
            }

            static void report(const ImuFiltered & imufilt)
            {
                static Helper _helper;

                if (_helper.ready()) {

                    const auto g = imufilt.gyroDps;
                    const auto a = imufilt.accelGs;

                    printf("%5lu | gx=%+08.3f gy=%+08.3f gz=%+08.3f dps | "
                           "ax=%+08.3f ay=%+08.3f az=%+08.3f Gs\n",
                            _helper.count, g.x, g.y, g.z, a.x, a.y, a.z);
                }
            }

            static void report(const ImuRaw & imuraw)
            {
                static Helper _helper;

                if (_helper.ready()) {

                    const auto gyro = imuraw.gyro;
                    const auto accel = imuraw.accel;

                    printf("%5lu | gx=%+05d gy=%+05d gz=%+05d | "
                            "ax=%+05d ay=%+05d az=%+05d\n",
                            _helper.count, gyro.x, gyro.y, gyro.z,
                            accel.x, accel.y, accel.z);
                }
            }
    };
}
