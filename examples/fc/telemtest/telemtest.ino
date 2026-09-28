#include <hackflight.h>
#include <firmware/fc.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/serializer.hpp>

static hf::MspSerializer telemetry_serializer_;

void setup()
{
    Serial1.begin(115200);
}

void loop()
{
    const float psi[1] = {99};

    telemetry_serializer_ = hf::MspSerializer::SerializeFloats(
            telemetry_serializer_, 101, psi, 1);

    /*
       Serial1.write(hf::MspSerializer::GetPayloadBytes(telemetry_serializer_),
       hf::MspSerializer::GetPayloadSize(telemetry_serializer_));*/

    const auto bytes = hf::MspSerializer::GetPayloadBytes(telemetry_serializer_);
    for (size_t k=0;
            k<hf::MspSerializer::GetPayloadSize(telemetry_serializer_); ++k) {
        Serial1.write(bytes[k]);
    }


    /*
       static uint8_t k_;
       const uint8_t c = 'A' + k_;
       Serial1.write(c);
       k_ = (k_ + 1) % 26;
     */

    //delay(1);
}
