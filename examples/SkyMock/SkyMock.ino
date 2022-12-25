#include "hackflight.h"
#include "msp/parser.h"

void setup(void)
{
    Serial.begin(115200);
}

void loop(void)
{
    static MspParser _parser;
    static MspSerializer _serializer;

    while (Serial.available()) {

        auto messageType = _parser.parse(Serial.read());

        int16_t x = -1;
        int16_t y = +1;

        switch (messageType) {

            case 121: // PAA3905

                _serializer.prepareToSerializeShorts(messageType, 2);
                _serializer.serializeShort(x);
                _serializer.serializeShort(y);
                _serializer.completeSerialize();

                Serial.write(_serializer.outBuf, _serializer.outBufSize);

                break;
        }
    }
}
