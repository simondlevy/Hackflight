/*
   Superfly Hackable ESP8266 Flight Controller implementation of Hackflight Board routines

   Uses EM7180 SENtral Sensor Hub in master mode mode

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include "boards/realboards/arduino.hpp"
#include "imus/usfs.hpp"

namespace hf {

    USFS superflyIMU;

    class SuperflyMotor : public Motor {

        private:

            static constexpr uint8_t PINS[4] = {4, 5, 12, 14};

        public:

            SuperflyMotor(void) : Motor(PINS, 4)
            {
            }

        protected:

            virtual void write(uint8_t index, float value) override
            {
                analogWrite(_pins[index], (uint16_t)(value * 1023));
            }

            virtual void begin(void) override
            {
                for (uint8_t k=0; k<_count; ++k) {
                    analogWrite(_pins[k], 0);  
                }
            }

    }; // class SuperflyMotor

    SuperflyMotor superflyMotors;

    class SuperFly : public ArduinoBoard {

        public:

            SuperFly(void) 
                : ArduinoBoard(15)
            {
                // Start I^2C
                Wire.begin(0,2); // SDA (0), SCL (2) on ESP8266

                // Hang a bit before starting up the EM7180
                delay(100);

                // Initialize the motors
                analogWriteFreq(200);  

                // Hang a bit more
                delay(100);
            }

    }; // class SuperFly

} // namespace hf
