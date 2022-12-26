#include "hackflight.h"
#include "msp/parser.h"
#include "msp/serializer/usb.h"

void setup(void)
{
    Serial.begin(115200);
}

void loop(void)
{
    static MspParser _parser;
    static UsbMspSerializer _serializer;

    while (Serial.available()) {

        auto messageType = _parser.parse(Serial.read());

        int16_t x = -1;
        int16_t y = +1;

        switch (messageType) {

            case 122: // PAA3905

                int16_t msg[] = {x, y};

                _serializer.serializeShorts(messageType, msg, 2);

                break;
        }
    }
}
