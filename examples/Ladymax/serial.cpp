#include <Arduino.h>
 
#include <serial.h>

serialReceiveCallbackPtr _rxCallback;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _rxCallback(Serial1.read(), NULL, micros());
    }
}

void serialBeginWrite(void * port)
{
    (void)port;
}

void serialEndWrite(void * port)
{
    (void)port;
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

uint8_t serialRead(void  * p)
{
    return p == &Serial ? 
        Serial.read() :
        p == &Serial1 ?
        Serial1.read() :
        0;
}

uint32_t serialRxBytesWaiting(void * port)
{
    (void)port;
    return 0;
}

uint32_t serialTxBytesFree(void * port)
{
    (void)port;
    return 0;
}

void serialWaitForPortToFinishTransmitting(void * port)
{
    (void)port;
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
