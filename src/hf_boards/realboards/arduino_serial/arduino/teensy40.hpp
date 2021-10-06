/*
   Teensy 4.0 implementation of Board routines

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "../arduino.hpp"

namespace hf {

    class Teensy40 : public ArduinoBoard {

         public:

            Teensy40(HardwareSerial * telemetryPort=NULL) 

                : ArduinoBoard(telemetryPort) 
            {
            }

    }; // class Teensy40

} // namespace hf
