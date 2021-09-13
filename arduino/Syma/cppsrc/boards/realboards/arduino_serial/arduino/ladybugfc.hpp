/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include "../../../../Hackflight.hpp"
#include "../arduino.hpp"

namespace hf {

    class LadybugFC : public hf::ArduinoBoard {

        public:

            LadybugFC(HardwareSerial * telemetryPort=NULL) 
                : hf::ArduinoBoard(false, telemetryPort) 
            {
            }

            void begin(void)
            {
                hf::ArduinoBoard::begin();

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class LadybugFC

} // namespace hf
