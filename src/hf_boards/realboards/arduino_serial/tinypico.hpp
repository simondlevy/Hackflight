/*
   TinyPICO implementation of Board routines, with IMU mounted on bottom of board

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Wire.h>
#include <TinyPICO.h>

#include "../arduino_serial.hpp"

namespace rft {

    class TinyPico : public ArduinoSerial {

        private:

            TinyPICO tp;

        protected:

            void setLed(bool isOn) 
            { 
                tp.DotStar_SetPixelColor(0, isOn?255:0, 0);
            }

        public:

            TinyPico(HardwareSerial * serial = &Serial1)
                : ArduinoSerial(serial)
            {
            }

    }; // class TinyPico

} // namespace rft
