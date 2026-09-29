#include <hackflight.h>
#include <firmware/espnow.hpp>
#include <firmware/blink_timer.hpp>
#include <firmware/msp/__messages__.h>
#include <firmware/msp/serializer.hpp>
#include <firmware/pushbutton.hpp>
#include <firmware/voltage_divider.hpp>

// Hardware-dependent --------------------------------------------------------

static const uint8_t kReceiverAddress[6] = {
    
    // OMGS3
    //0x98,0x3D,0xAE,0xEF,0x0E,0xAC

    // TinyS3
    // 0xB4, 0x3A, 0x45, 0xB2, 0x09, 0x2C
    0xB4, 0x3A, 0x45, 0xB1, 0xF1, 0xC0
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
