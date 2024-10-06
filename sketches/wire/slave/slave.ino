#include "Wire.h"

#define I2C_DEV_ADDR 0x55

void onRequest() 
{
    static uint32_t count;
    Wire1.printf(" Packets: %u", count++);
}

void setup() 
{
    Wire1.onRequest(onRequest);

    Wire1.begin((uint8_t)I2C_DEV_ADDR);
}

void loop() 
{
}
