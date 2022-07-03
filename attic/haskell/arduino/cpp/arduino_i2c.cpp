/*
   Arduino I^2C support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

void i2cStart(void)
{
    Wire.begin();
    delay(100);
}
