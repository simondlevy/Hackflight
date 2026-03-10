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

namespace hf {

    class Debugger {

        public:

            static void debug(const VehicleState & state)
            {
                static uint32_t _count;
                static uint32_t _msec;
                const auto msec = millis();

                if (msec - _msec > 10) {

                    printf("%5lu | dx=%+3.3f dy=%+3.3f z=%3.3f dz=%+3.3f "
                            "phi=%+08.3f dphi=%+08.3f theta=%+08.3f dtheta=%+08.3f"
                            " psi=%+08.3f dpsi=%+08.3f\n",
                            _count++, state.dx, state.dy, state.z, state.dz,
                            state.phi, state.dphi, state.theta, state.dtheta,
                            state.psi, state.dpsi);

                    _msec = msec;
                }
            }

            static void debug(const ImuFiltered & imufilt)
            {
                static uint32_t _count;
                static uint32_t _msec;
                const auto msec = millis();

                if (msec - _msec > 10) {

                    const auto g = imufilt.gyroDps;
                    const auto a = imufilt.accelGs;

                    printf("%5lu | gx=%+3.3f gy=%+3.3f gz=%+3.3f dps | "
                            "ax=%+3.3f ay=%+3.3f az=%+3.3f Gs\n",
                            _count++, g.x, g.y, g.z, a.x, a.y, a.z);

                    _msec = msec;
                }
            }

            static void profile()
            {
                static uint32_t _msec;
                const auto msec = millis();
                static uint32_t _count;

                if (msec - _msec > 1000) {
                    if (_count > 0) {
                        printf("count=%d\n", (int)_count);
                    }
                    _msec = msec;
                    _count = 0;
                }
                _count++;
            }
    };
}
