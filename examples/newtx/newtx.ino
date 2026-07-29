#include <hackflight.h>
#include <firmware/espnow.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const uint8_t kArmingDigitalPin = 35;
static const uint8_t kHoverDigitalPin = 9;
static const uint8_t kAutopilotDigitalPin = 8;

void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

    pinMode(kArmingDigitalPin, INPUT);
    pinMode(kHoverDigitalPin, INPUT);
    pinMode(kAutopilotDigitalPin, INPUT);
}

void loop()
{
    Serial.printf(
            "Arm=%d "
            "Hover=%d "
            "Autopilot=%d "
            "A0=%04d "
            "A1=%04d "
            "A2=%04d "
            "A3=%04d "
            "A4=%04d "
            "\n"
            , digitalRead(kArmingDigitalPin)
            , digitalRead(kHoverDigitalPin)
            , digitalRead(kAutopilotDigitalPin)
            , analogRead(A0)
            , analogRead(A1)
            , analogRead(A2)
            , analogRead(A3)
            , analogRead(A4)
            );

}
