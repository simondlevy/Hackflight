#include <hackflight.h>
#include <firmware/espnow.hpp>
#include <firmware/blink_timer.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/serializer.hpp>
#include <firmware/pushbutton.hpp>
#include <firmware/voltage_divider.hpp>

// Hardware-dependent --------------------------------------------------------

static const uint8_t kReceiverAddress[6] = {
    
    // TinyPICO
    0xD4, 0xD4, 0xDA, 0xAA, 0x2E, 0xF0
};


void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);
}

void loop()
{
    const short vals[7] = {};

    static hf::MspSerializer serializer_;

    serializer_ = hf::MspSerializer::SerializeShorts(
            serializer_, kMspSetChannels, vals, 7);

    const auto result = esp_now_send(kReceiverAddress,
            hf::MspSerializer::GetPayloadBytes(serializer_),
            hf::MspSerializer::GetPayloadSize(serializer_));

    if (result != ESP_OK) {
        Serial.printf("ERROR sending to vehicle: %d\n", result);
    }

    delay(10);
}
