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
                return OpticalFlowRaw(0, 0, false);
            }

        private:

            PMW3901 _pmw3901;
    };
}

#if 0

void loop() 
{
    int16_t deltaX = 0;
    int16_t deltaY = 0;
    bool gotMotion = false;

    sensor.readMotion(deltaX, deltaY, gotMotion); 

    Serial.print("deltaX: ");
    Serial.print(deltaX);
    Serial.print(",\tdeltaY: ");
    Serial.print(deltaY);
    Serial.print(",\tgotMotion: ");
    Serial.println(gotMotion ? "yes" : "no");

    delay(100);
}
#endif
