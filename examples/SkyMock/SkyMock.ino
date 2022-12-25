#include "hackflight.h"
#include "msp/parser.h"

void setup(void)
{
    Serial.begin(115200);
}

void loop(void)
{
    static MspParser _parser;

    while (Serial.available()) {

        auto messageType = _parser.parse(Serial.read());
    }
}
