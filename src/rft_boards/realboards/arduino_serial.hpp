/*
   Class for Arduino-style serial comms

   Support communication over Serial (USB) and
   telemetry port. 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include "../realboard.hpp"

namespace rft {

    class ArduinoSerial : public RealBoard {

        private:

            HardwareSerial * _telemetryPort = NULL;

        protected:

            ArduinoSerial(HardwareSerial * telemetryPort=NULL)
            {
                _telemetryPort = telemetryPort;
            }

            uint8_t serialAvailable(bool useTelemetryPort)
            {
                if (useTelemetryPort) {
                    return _telemetryPort ? _telemetryPort->available() : 0;
                }

                return Serial.available();
            }

            uint8_t serialRead(bool useTelemetryPort)
            {
                if (useTelemetryPort) {
                    return _telemetryPort ? _telemetryPort->read() : 0;
                }

                return Serial.read();
            }

            void serialWrite(uint8_t byte, bool useTelemetryPort)
            {
                if (useTelemetryPort) {
                    if (_telemetryPort) {
                        _telemetryPort->write(byte);
                    }
                }

                else {
                    Serial.write(byte);
                }
            }

            void begin(void)
            {
                // Start serial communcation for GCS/debugging
                Serial.begin(SERIAL_BAUD);

                // Start serial communication for telemetry if provided
                if (_telemetryPort) {
                    _telemetryPort->begin(SERIAL_BAUD);
                }

                // This will blink the LED
                RealBoard::begin();
            }

    }; // class ArduinoSerial

    void Debugger::outbuf(char * buf)
    {
        Serial.print(buf);
    }

} // namespace rft
