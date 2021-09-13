/*
   Superclass for Arduino-based flight controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
*/

#pragma once

#include "../arduino_serial.hpp"
#include "../../../../copilot.h"

namespace hf {

    class ArduinoBoard : public ArduinoSerial {

        private:

            bool _led_inverted = false;

        protected:

            ArduinoBoard(bool ledInverted=false, HardwareSerial * telemetryPort=NULL)
                : ArduinoSerial(telemetryPort)
            {
                _led_inverted= ledInverted;
            }

            void begin(void)
            {
                copilot_setLed(_led_inverted ? HIGH : LOW);
                ArduinoSerial::begin();
            }

            void setLed(bool isOn) 
            { 
                copilot_setLed(isOn ?
                        (_led_inverted?LOW:HIGH) :
                        (_led_inverted?HIGH:LOW));
            }

    }; // class ArduinoBoard

} // namespace hf
