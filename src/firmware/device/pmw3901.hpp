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


// Standard Arduino libraries
#include <SPI.h>

// Third-party libraries
#include <pmw3901.hpp>

// Hackflight library
#include <hackflight.h>
#include <firmware/device/debugging.hpp>
#include <firmware/device/datatypes.hpp>

namespace hf {

    class OpticalFlowSensor {

        public:

            void begin()
            {
                SPI.begin();

                if (!_pmw3901.begin()) {
                    Debugger::reportForever("Unable to initialize PMW3901");
                }
            }


            auto read() -> OpticalFlowRaw
            {
                int16_t dx = 0;
                int16_t dy = 0;
                auto moved = false;

                _pmw3901.readMotion(dx, dy, moved);

                return OpticalFlowRaw(dx, dy, moved);
            }

        private:

            PMW3901 _pmw3901;
    };
}
