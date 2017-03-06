#include <hackflight.hpp>

#include <Arduino.h>
#include "naze.hpp"

hf::Hackflight h;

void setup(void)
{
    Serial.begin(115200);
    h.init(new hf::Naze());
}

void loop(void)
{
    Serial.printf("%ld\n", millis());
}
