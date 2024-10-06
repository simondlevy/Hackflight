#include "Wire.h"

#define I2C_DEV_ADDR 0x55

uint32_t i = 0;

void setup() 
{
    Serial.begin(115200);
    Wire.begin();
}

void loop() 
{
    delay(100);

    uint8_t bytesReceived = Wire.requestFrom(I2C_DEV_ADDR, 16);

    if (bytesReceived > 0) {  
        uint8_t temp[bytesReceived];
        Wire.readBytes(temp, bytesReceived);
        Serial.printf("%s\n", (char *)temp);
    }
}
