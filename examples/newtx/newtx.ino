#include <hackflight.h>
#include <firmware/espnow.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const uint8_t kThrottleAnalogPin = A0;
static const uint8_t kRollAnalogPin = A2;
static const uint8_t kPitchAnalogPin = A3;
static const uint8_t kYawAnalogPin = A1;
static const uint8_t kDividerAnalogPin = A4;

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
            "Throttle=%04d "
            "Roll=%04d "
            "Pitch=%04d "
            "Yaw=%04d "
            "Divider=%04d "
            "\n"
            , digitalRead(kArmingDigitalPin)
            , digitalRead(kHoverDigitalPin)
            , digitalRead(kAutopilotDigitalPin)
            , analogRead(kThrottleAnalogPin)
            , analogRead(kRollAnalogPin)
            , analogRead(kPitchAnalogPin)
            , analogRead(kYawAnalogPin)
            , analogRead(kDividerAnalogPin)
            );

}
