/*
   mspsensor.hpp : Abstract class for sensors using MSP

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

#include "sensor.hpp"
#include "mspparser.hpp"
#include "boards/realboard.hpp"

namespace hf {

    class MspSensor : public Sensor, MspParser {

        friend class Hackflight;

        private:

        // Sensor must be able to access boards' UART
        RealBoard * _board;

        protected:

        MspSensor(RealBoard * board)
        {
            _board = board;
        }

        virtual bool ready(float time) override
        {
            (void)time;

            bool retval = false;

            while (_board->serialTelemetryAvailable()) {
                MspParser::parse(_board->serialTelemetryRead());
            }

            return retval;
        }

        public:

        void init(void) 
        {
            MspParser::init();
        }

    };  // class MspSensor

} // namespace
