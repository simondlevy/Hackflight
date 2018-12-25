/*
   dragonfly.hpp : Board subclass for Arduino prototyping without IMU or motors

   Copyright (c) 2018 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
   */

#pragma once

#include "mock.hpp"

namespace hf {

    class DragonflyBoard : public MockBoard {

        protected:

            uint8_t serialTelemetryAvailable(void) override
            {
                return Serial1.available();
            }

            uint8_t serialTelemetryRead(void) override
            {
                return Serial1.read();
            }

            void serialTelemetryWrite(uint8_t c) override
            {
                Serial1.write(c);
            }

        public:

            DragonflyBoard(void) : MockBoard(25, true)
            {
            }

    }; // class DragonflyBoard

} // namespace hf
