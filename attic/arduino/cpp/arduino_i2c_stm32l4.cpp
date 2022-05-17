/*
   Arduino I^2C support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>
#include <Wire.h>

void stream_startI2C(void)
{
    Wire.begin();
    delay(100);
}

void stream_startI2C_STM32L4_PINS_6_7(void)
{
    Wire.begin(TWI_PINS_6_7);
    delay(100);
}
