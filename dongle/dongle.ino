#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    static uint32_t count;

    Serial.println(count++);

    delay(1000);
}
