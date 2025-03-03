#include "bootloader.h"

void setup()
{
    Serial.begin(115200);
}

void loop()
{  
    static uint32_t count = 0;

    Serial.println(count++);

    delay(1000);
}
