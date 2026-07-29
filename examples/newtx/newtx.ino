#include <hackflight.h>
#include <firmware/espnow.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const uint8_t DIGITAL_PIN_1 = 35;

void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

    pinMode(DIGITAL_PIN_1, INPUT);
}

void loop()
{
    Serial.printf(
            "D%d=%d "
            "A0=%04d "
            "A1=%04d "
            "A2=%04d "
            "A3=%04d "
            "A4=%04d "
            "\n"
            , DIGITAL_PIN_1, digitalRead(DIGITAL_PIN_1)
            , analogRead(A0)
            , analogRead(A1)
            , analogRead(A2)
            , analogRead(A3)
            , analogRead(A4)
            );

}
