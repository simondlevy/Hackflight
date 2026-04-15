#include "../espnow_helper.hpp"

static const uint8_t XIAO_ADDRESS[] = {0x8C,0xBF,0xEA,0xCB,0x8F,0x94};

static EspNowHelper _helper = EspNowHelper(XIAO_ADDRESS);

void setup()
{
    Serial.begin(115200);

    _helper.begin();
}

void loop()
{
    const uint8_t vals[10] = {};

    _helper.send(vals, 10);

    delay(100);
}
