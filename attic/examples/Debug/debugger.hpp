/*
   Serial debugging support for Hackflight

   Provides printf() and related static methods for formatted printing of
   debug messages.  

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <stdarg.h>
#include <stdio.h>

#include <Arduino.h>

class Debugger {

    public:

        static void printf(HardwareSerial & serial, const char * fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            char buf[200];
            vsnprintf(buf, 200, fmt, ap); 
            serial.print(buf);
            va_end(ap);
        }

        static void printf(const char * fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            char buf[200];
            vsnprintf(buf, 200, fmt, ap); 
            Serial.print(buf);
            va_end(ap);
        }

}; // class Debugger
