/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include <RoboFirmwareToolkit.hpp>

#include <rft_boards/realboards/arduino_serial/arduino.hpp>

namespace hf {

    class LadybugFC : public rft::ArduinoBoard {

        public:

            // Support prototype version where LED is on pin A1
            LadybugFC(uint8_t ledPin = A4) 
                : rft::ArduinoBoard(ledPin)
            {
            }

            void begin(void)
            {
                rft::ArduinoBoard::begin();

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class LadybugFC

} // namespace hf
