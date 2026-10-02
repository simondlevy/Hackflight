#include <hackflight.h>
#include <firmware/espnow.hpp>
#include <firmware/blink_timer.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/serializer.hpp>
#include <firmware/pushbutton.hpp>
#include <firmware/voltage_divider.hpp>

// Hardware-dependent --------------------------------------------------------

static const uint8_t kDongleAddress[6] = {
    
    // TinyPICO
    0x00, 0x4B, 0x12, 0xCD, 0x9B, 0xD0
};


void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kDongleAddress);
}

void loop()
{
    const short vals[7] = {};

    static hf::MspSerializer serializer_;

    serializer_ = hf::MspSerializer::SerializeShorts(
            serializer_, kMspSetChannels, vals, 7);

    const auto result = esp_now_send(kDongleAddress,
            hf::MspSerializer::GetPayloadBytes(serializer_),
            hf::MspSerializer::GetPayloadSize(serializer_));

    if (result != ESP_OK) {
        Serial.printf("ERROR sending to dongle: %d\n", result);
    }

    delay(10);
}
