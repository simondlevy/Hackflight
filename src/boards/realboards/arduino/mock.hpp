/*
   Board subclass for Arduino prototyping without IMU or motors

   Copyright (c) 2021 Simon D. Levy

   MIT License
*/

#pragma once

#include "boards/realdboards/arduino.hpp"

namespace hf {

    class MockBoard : public ArduinoBoard {

        protected:


            uint8_t serialTelemetryAvailable(void) override
            {
                return Serial1.available();
            }

            uint8_t serialTelemetryRead(void) override
            {
                return Serial1.read();
            }

            void serialTelemetryWrite(uint8_t c) override
            {
                Serial1.write(c);
            }

        public:

            MockBoard(uint8_t ledPin, bool ledInverted=false) 
                : ArduinoBoard(ledPin, ledInverted)
            {

                // Set up to receive telemetry over Serial1
                Serial1.begin(115200);
                _time = 0;
            }

    }; // class MockBoard

} // namespace hf
