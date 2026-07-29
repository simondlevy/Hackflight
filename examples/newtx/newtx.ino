/* Hackflight ESP32 transmitter sketch
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

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

static bool arming_prev_;

void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

    pinMode(kArmingDigitalPin, INPUT);
    pinMode(kHoverDigitalPin, INPUT);
    pinMode(kAutopilotDigitalPin, INPUT);

    arming_prev_ = digitalRead(kArmingDigitalPin);
}

void loop()
{
    static bool armed_;
    const auto arming_curr = digitalRead(kArmingDigitalPin);
    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }
    arming_prev_ = arming_curr;

    Serial.printf(
            "Armed=%d "
            "Hovering=%d "
            "Autopilot=%d "
            "Throttle=%04d "
            "Roll=%04d "
            "Pitch=%04d "
            "Yaw=%04d "
            "Divider=%04d "
            "\n"
            , armed_
            , digitalRead(kHoverDigitalPin)
            , digitalRead(kAutopilotDigitalPin)
            , analogRead(kThrottleAnalogPin)
            , analogRead(kRollAnalogPin)
            , analogRead(kPitchAnalogPin)
            , analogRead(kYawAnalogPin)
            , analogRead(kDividerAnalogPin)
            );

}
