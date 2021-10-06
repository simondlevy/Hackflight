/*
   Superclass for Arduino-based flight controllers

   Copyright (c) 2021 Simon D. Levy

   MIT License
*/

#pragma once

#include "../arduino_serial.hpp"

namespace rft {

    class ArduinoBoard : public ArduinoSerial {

        private:

            uint8_t _led_pin = 0;
            bool _led_inverted = false;

            static void powerPin(uint8_t id, uint8_t value)
            {
                pinMode(id, OUTPUT);
                digitalWrite(id, value);
            }

        protected:

            ArduinoBoard(int8_t ledPin=13,
                         HardwareSerial * telemetryPort=NULL)
                : ArduinoSerial(telemetryPort)
            {
                _led_pin = abs(ledPin);
                _led_inverted = ledPin < 0;

            }

            void begin(void)
            {
                if (_led_pin) {
                    pinMode(_led_pin, OUTPUT);
                    digitalWrite(_led_pin, _led_inverted ? HIGH : LOW);
                }

                ArduinoSerial::begin();
            }

            void setLed(bool isOn) 
            { 
                if (_led_pin) {
                    digitalWrite(_led_pin, isOn ?
                            (_led_inverted?LOW:HIGH) :
                            (_led_inverted?HIGH:LOW));
                }
            }

        public:

            static void powerPins(uint8_t pwr, uint8_t gnd)
            {
                powerPin(pwr, HIGH);
                powerPin(gnd, LOW);
            }

    }; // class ArduinoBoard

} // namespace rft
