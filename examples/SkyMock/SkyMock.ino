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

        int16_t p11 = 11;
        int16_t p12 = 12;
        int16_t p13 = 13;
        int16_t p14 = 14;
        int16_t p21 = 21;
        int16_t p22 = 22;
        int16_t p23 = 23;
        int16_t p24 = 24;
        int16_t p31 = 31;
        int16_t p32 = 32;
        int16_t p33 = 33;
        int16_t p34 = 34;
        int16_t p41 = 41;
        int16_t p42 = 42;
        int16_t p43 = 43;
        int16_t p44 = 44;

        int16_t vmsg[] = {p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34, p41, p42, p43, p44};

        int16_t pmsg[] = {x, y};

        switch (messageType) {

            case 121:   // VL53L5
                _serializer.serializeShorts(messageType, vmsg, 16);
                break;

            case 122: // PAA3905
                _serializer.serializeShorts(messageType, pmsg, 2);
                break;
        }
    }
}
