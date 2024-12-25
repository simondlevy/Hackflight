#include <hackflight.hpp>
#include <receivers/esp32.hpp>

static hf::Esp32Receiver _rx;

void setup() 
{
    Serial.begin(115200);

    _rx.begin();
}

void loop() 
{
    static uint16_t chan[6];
    static bool failsafe;

    _rx.read(chan, failsafe);

    printf("%ld\n", chan[0]);
}
