#include <Arduino.h>
 
#include <serial.h>

serialReceiveCallbackPtr _rxCallback;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _rxCallback(Serial1.read(), NULL, micros());
    }
}

bool serialIsTransmitBufferEmpty(void * port)
{
    // Always use Serial
    return Serial.availableForWrite() > 0;
}

void serialOpenPortSbus( serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback)
{
    // Always use Serial1
    (void)identifier;
    Serial1.begin(115200);

    _rxCallback = rxCallback;
}

void * serialOpenPortUsb(void)
{
    Serial.begin(115200);

    return &Serial;
}

uint8_t serialRead(void  * port)
{
    return port == &Serial ? 
        Serial.read() :
        port == &Serial1 ?
        Serial1.read() :
        0;
}

uint32_t serialBytesAvailable(void * port)
{
    return port == &Serial ? 
        Serial.available() :
        port == &Serial1 ?
        Serial1.available() :
        0;
}

void serialWrite(void * port, uint8_t c)
{
    (void)port;
    (void)c;
}

void serialWriteBuf(void * port, const uint8_t *data, uint32_t count)
{
    (void)port;
    (void)data;
    (void)count;
}
