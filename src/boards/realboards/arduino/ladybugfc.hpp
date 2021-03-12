/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "boards/realboards/arduino.hpp"
#include "imus/usfs.hpp"
#include "motors/brushed.hpp"

namespace hf {

    USFS ladybugIMU;

    static const uint8_t MOTOR_PINS[4] = {13, A2, 3, 11};

    BrushedMotor ladybugFcNewMotors = BrushedMotor(MOTOR_PINS, 4);

    class LadybugFC : public ArduinoBoard {

        public:

            // Support prototype version where LED is on pin A1
            LadybugFC(uint8_t ledPin = A4) 
                : ArduinoBoard(ledPin)
            {
                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class LadybugFC

} // namespace hf
