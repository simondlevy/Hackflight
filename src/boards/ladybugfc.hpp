/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <rft_boards/realboards/arduino.hpp>

#include <Wire.h>

namespace hf {

    class LadybugFC : public rft::ArduinoBoard {

        protected:

            void begin(void)
            {
                rft::ArduinoBoard::begin();

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

        public:

            static const uint8_t MOTOR1_PIN = 13;
            static const uint8_t MOTOR2_PIN = A2;
            static const uint8_t MOTOR3_PIN = 3;
            static const uint8_t MOTOR4_PIN = 11;

            // Support prototype version where LED is on pin A1
            LadybugFC(uint8_t ledPin = A4) 
                : rft::ArduinoBoard(ledPin)
            {
            }

    }; // class LadybugFC

} // namespace hf
