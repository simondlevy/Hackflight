#include <Arduino.h>

#include <hackflight.hpp>
#include "beef.hpp"

hf::Hackflight h;

void setup(void)
{
    Serial.begin(115200);
    h.init(new hf::Beef());
}

void loop(void)
{
    Serial.printf("%d\n", millis());
    h.update();
}
