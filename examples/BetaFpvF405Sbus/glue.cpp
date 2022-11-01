#include <Arduino.h>

uint32_t serialBytesAvailable(void * port)
{
    (void)port;
    return Serial.available();
}

void * serialOpenPortUsb(void)
{
    Serial.begin(115200);
    return NULL;
}

uint8_t serialRead(void * port)
{
    (void)port;
    return Serial.read();
} 

void serialWriteBuf(void * port, const uint8_t *data, uint32_t count)
{
    (void)port;
    Serial.write(data, count);
}
