#include "../espnow_helper.hpp"

static const uint8_t DONGLE_ADDRESS[] = {0xD4,0xD4,0xDA,0x83,0x97,0x90};

static EspNowHelper _helper = EspNowHelper(DONGLE_ADDRESS);

void setup()
{
    Serial.begin(115200);

    _helper.begin();
}

void loop()
{
    const uint8_t vals[5] = {};

    _helper.send(vals, 5);

    delay(100);
}
