#include <hackflight.h>
#include <firmware/espnow.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const uint8_t DPIN1 = 35;
static const uint8_t DPIN2 = 9;

void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

    pinMode(DPIN1, INPUT);
    pinMode(DPIN2, INPUT);
}

void loop()
{
    Serial.printf(
            "D%d=%d "
            "D%d=%d "
            "A0=%04d "
            "A1=%04d "
            "A2=%04d "
            "A3=%04d "
            "A4=%04d "
            "\n"
            , DPIN1, digitalRead(DPIN1)
            , DPIN2, digitalRead(DPIN2)
            , analogRead(A0)
            , analogRead(A1)
            , analogRead(A2)
            , analogRead(A3)
            , analogRead(A4)
            );

}
