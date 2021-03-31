/*
   Ladybug Brushed Flight Controller implementation of Hackflight Board routines

   Uses USFS Sensor Hub in master mode mode

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>

#include <rft_boards/realboards/arduino.hpp>
#include <rft_motors/realmotors/brushed.hpp>

#include "sensors/usfs.hpp"

namespace hf {

    class LadybugFC : public rft::ArduinoBoard {

        private:

            static constexpr uint8_t MOTOR_PINS[4] = {13, A2, 3, 11};

        public:

            rft::BrushedMotor motors = rft::BrushedMotor(MOTOR_PINS, 4);

            UsfsGyro gyro;
            UsfsQuat quat;

            LadybugFC(void)
                : ArduinoBoard(A4)
            {
            }

            void begin(void) override 
            {
                rft::ArduinoBoard::begin();

                // Start I^2C
                Wire.begin();

                // Hang a bit
                delay(100);
            }

    }; // class LadybugFC

} // namespace hf
