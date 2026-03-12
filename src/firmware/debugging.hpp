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

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/timer.hpp>

namespace hf {

    class Debugger {

        private:

            static constexpr float FREQ = 100;

        public:

            static void debug(const float * chanvals, const bool is_armed)
            {
                static Timer _timer;

                if (_timer.ready(FREQ)) {

                    static uint32_t _count;

                    printf("%5lu | armed=%d | throt=%+3.3f roll=%+3.3f "
                            "pitch=%3.3f yaw=%+3.3f\n",
                            _count++, is_armed, chanvals[0], chanvals[1],
                            chanvals[2], chanvals[3]); }
            }

             static void debug(const Setpoint & setpoint)
            {
                static Timer _timer;

                if (_timer.ready(FREQ)) {

                    static uint32_t _count;

                    printf("%5lu | thrust=%+3.3f roll=%+3.3f pitch=%3.3f yaw=%+3.3f\n",
                            _count++, setpoint.thrust, setpoint.roll,
                            setpoint.pitch, setpoint.yaw); }
            }

            static void debug(const VehicleState & state)
            {
                static Timer _timer;

                if (_timer.ready(FREQ)) {

                    static uint32_t _count;

                    printf("%5lu | dx=%+8.3f dy=%+8.3f z=%8.3f dz=%+8.3f "
                            "phi=%+08.3f dphi=%+08.3f theta=%+08.3f dtheta=%+08.3f"
                            " psi=%+08.3f dpsi=%+08.3f\n",
                            _count++, state.dx, state.dy, state.z, state.dz,
                            state.phi, state.dphi, state.theta, state.dtheta,
                            state.psi, state.dpsi);
                }
            }

            static void debug(const ImuFiltered & imufilt)
            {
                static Timer _timer;

                if (_timer.ready(FREQ)) {

                    static uint32_t _count;

                    const auto g = imufilt.gyroDps;
                    const auto a = imufilt.accelGs;

                    printf("%5lu | gx=%+3.3f gy=%+3.3f gz=%+3.3f dps | "
                            "ax=%+3.3f ay=%+3.3f az=%+3.3f Gs\n",
                            _count++, g.x, g.y, g.z, a.x, a.y, a.z);
                }
            }

            static void debug(const ImuRaw & imuraw)
            {
                static Timer _timer;

                if (_timer.ready(FREQ)) {

                    static uint32_t _count;

                    const auto gyro = imuraw.gyro;
                    const auto accel = imuraw.accel;

                    printf("%5lu | gx=%+05d gy=%+05d gz=%+05d | "
                            "ax=%+05d ay=%+05d az=%+05d\n",
                            _count++, gyro.x, gyro.y, gyro.z,
                            accel.x, accel.y, accel.z);
                }
            }

            static void profile()
            {
                static Timer _timer;

                static uint32_t _count;

                if (_timer.ready(1)) {

                     if (_count > 0) {
                        printf("count=%d\n", (int)_count);
                    }

                    _count = 0;
                }
                _count++;
            }
    };
}
