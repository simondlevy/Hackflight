#include <Arduino.h>
#include <Wire.h>

#include <hackflight.h>

static hackflight_t hf;

void setup(void)
{
    Wire.begin();
    delay(100);
    Serial.begin(115200);

    hackflightFullInit(&hf);
}

void loop(void)
{
}

