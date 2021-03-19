/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses USFS Sensor Hub in master mode mode

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include <rft_boards/realboards/arduino.hpp>
#include <rft_motors/brushed.hpp>

#include "sensors/usfs.hpp"

namespace hf {

    class LadybugFC : public rft::ArduinoBoard {

        public:

            LadybugFC(void)
                : ArduinoBoard(A4)
            {
                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class LadybugFC

} // namespace hf
