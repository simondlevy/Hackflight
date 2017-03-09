#include <Arduino.h>

#include <hackflight.hpp>
#include "naze.hpp"

hf::Hackflight h;

void setup(void)
{
    Serial.begin(115200);
    h.init(new hf::Naze());
}

void loop(void)
{
    Serial.printf("%d\n", millis());
    //h.update();
}
