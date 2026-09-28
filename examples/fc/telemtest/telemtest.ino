#include <hackflight.h>
#include <firmware/fc.hpp>

static hf::FlightController fc_;

void setup()
{
    Serial3.begin(115200);
}

void loop()
{
    (void)fc_;

    const auto setpoint = hf::Setpoint(0, 0, 0, 0);

    /*
    static uint8_t k_;
    const uint8_t c = 'A' + k_;
    Serial3.write(c);
    k_ = (k_ + 1) % 26;*/

    fc_.SendTelemetry(Serial3, setpoint);

    delay(2);
}
