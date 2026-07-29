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
#include <firmware/switches/intermittent.hpp>
#include <firmware/switches/latching.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const uint8_t kThrottleAnalogPin = A0;
static const uint8_t kRollAnalogPin = A2;
static const uint8_t kPitchAnalogPin = A3;
static const uint8_t kYawAnalogPin = A1;
static const uint8_t kDividerAnalogPin = A4;

static auto armingSwitch_ = hf::LatchingSwitch(35);
static auto hoveringSwitch_ = hf::IntermittentSwitch(9);
static auto autopilotSwitch_ = hf::IntermittentSwitch(8);

void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

    armingSwitch_.Begin();
    hoveringSwitch_.Begin();
    autopilotSwitch_.Begin();
}

void loop()
{
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
            , armingSwitch_.Read()
            , hoveringSwitch_.Read()
            , autopilotSwitch_.Read()
            , analogRead(kThrottleAnalogPin)
            , analogRead(kRollAnalogPin)
            , analogRead(kPitchAnalogPin)
            , analogRead(kYawAnalogPin)
            , analogRead(kDividerAnalogPin)
            );

}
