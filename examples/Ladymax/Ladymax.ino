#include <Arduino.h>
#include <Wire.h>

#include <hackflight.h>
#include <serial.h>

static hackflight_t hf;

void setup(void)
{
    Wire.begin();
    delay(100);

    // Always use Serial1 for receiver, no no need to specify
    hackflightFullInit(&hf, SERIAL_PORT_NONE);
}

void loop(void)
{
}

