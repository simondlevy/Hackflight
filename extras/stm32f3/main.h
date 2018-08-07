/*
Arduino.h : Arduino API header for BreezySTM32 library

Copyright (C) 2017 Simon D. Levy 

This file is part of BreezySTM32.

BreezySTM32 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

BreezySTM32 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with BreezySTM32.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

extern "C" {

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t level);

unsigned long micros(void);
unsigned long millis(void);
void delay(unsigned long);

void reset(void);
void resetToBootloader(void);

void setup(void);
void loop(void);

class HardwareSerial {

    protected:

        void *  _uart;

    public:

        uint8_t available(void);

        uint8_t read(void);

        void write(uint8_t byte);

        void flush(void);

        void printf(const char * fmt, ...)
        {
            va_list ap;       
            va_start(ap, fmt);     
            char buf[1000];
            vsprintf(buf, fmt, ap);
            for (char *p=buf; *p; p++)
               this->write(*p);
            va_end(ap);  
            this->flush();
        }
};

class HardwareSerial0 : public HardwareSerial {

    public:

        void begin(uint32_t baud);

        uint8_t read(void);
};
extern HardwareSerial0 Serial;

class HardwareSerial1 : public HardwareSerial {

    public:

        void begin(uint32_t baud);

        uint8_t read(void);
};
extern HardwareSerial1 Serial1;

class HardwareSerial2 : public HardwareSerial {

    public:

        void begin(uint32_t baud);

        uint8_t read(void);
};
extern HardwareSerial2 Serial2;

class HardwareSerial3 : public HardwareSerial {

    public:

        void begin(uint32_t baud);

        uint8_t read(void);
};
extern HardwareSerial3 Serial3;


} // extern "C"
