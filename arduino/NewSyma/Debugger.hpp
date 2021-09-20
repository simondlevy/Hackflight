/*
   Serial debugging support

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <Arduino.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

class Debugger {

    private:

        HardwareSerial * _port = NULL;

    public:

        Debugger(HardwareSerial * port = &Serial)
        {
            _port = port;
        }

        void begin(void)
        {
            _port->begin(115200);
        }

        void printf(const char * fmt, ...)
        {
            va_list ap;
            va_start(ap, fmt);
            char buf[200];
            vsnprintf(buf, 200, fmt, ap); 

            _port->print(buf);
            _port->flush();

            va_end(ap);
        }

}; // class Debugger
