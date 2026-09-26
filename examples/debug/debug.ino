#include <hackflight.h>
#include <firmware/receivers/new.hpp>

static hf::NewReceiver rx_;

void serialEvent3()
{
    while (Serial3.available()) {

        rx_ = hf::NewReceiver::ParseByte(rx_, Serial3.read(), millis());
    }
}

void setup()
{
    Serial3.begin(115200);

    delay(3000);
}

void loop()
{
    rx_ = hf::NewReceiver::Update(rx_);

    printf("armed=%d\n", hf::NewReceiver::IsArmed(rx_));

    delay(1);
}
