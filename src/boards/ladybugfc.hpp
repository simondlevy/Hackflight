/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2018 Simon D. Levy

   MIT License
 */

#pragma once

#include <rft_boards/realboards/arduino.hpp>
#include <rft_motors/realmotors/brushed.hpp>

#include <Wire.h>

namespace hf {

    static const uint8_t MOTOR_PINS[4] = {13, A2, 3, 11};

    rft::BrushedMotor ladybugFcNewMotors = rft::BrushedMotor(MOTOR_PINS, 4);

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

            // Support prototype version where LED is on pin A1
            LadybugFC(uint8_t ledPin = A4) 
                : rft::ArduinoBoard(ledPin)
            {
            }

    }; // class LadybugFC

} // namespace hf
