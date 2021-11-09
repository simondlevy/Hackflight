#include <Arduino.h>
#include <Wire.h>

void stream_startMpu9250(void);
void stream_updateMpu9250(void);

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();
    delay(100);
    stream_startMpu9250();
}

void loop(void)
{
    stream_updateMpu9250();
}
