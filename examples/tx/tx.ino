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
#include <firmware/blink_timer.hpp>
#include <firmware/pushbuttons.hpp>
#include <firmware/voltage_divider.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const uint8_t kYawPin = A0;
static const uint8_t kThrottlePin = A1;
static const uint8_t kRollPin = A2;
static const uint8_t kPitchPin = A3;
static const uint8_t kVoltageDividerPin = A4;

static const uint8_t kLedPin = 33;

static const float kVoltageDividerR1Ohms = 1000;
static const float kVoltageDividerR2Ohms = 2200;

static const float kAnalogMin = 240;
static const float kAnalogMax = 3900;

static const float kLowVoltage = 3.0;

static const float kTransmitHz = 100;


static auto blink_timer_ = hf::BlinkTimer();

static auto transmit_timer_ = hf::Timer(kTransmitHz);

static hf::VoltageDivider voltage_divider_ = hf::VoltageDivider(
        kVoltageDividerPin,
        kVoltageDividerR1Ohms,
        kVoltageDividerR2Ohms,
        12);

static auto armingButton = hf::LatchingPushbutton(A1);
static auto hoveringButton = hf::IntermittentPushbutton(A8);
static auto autopilotButton = hf::IntermittentPushbutton(A7);

void setup()
{
    Serial.begin(115200);

    pinMode(kLedPin, OUTPUT);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

}

void loop()
{
    const auto volts = voltage_divider_.read();

    digitalWrite(kLedPin, volts < kLowVoltage ? blink_timer_.On() : HIGH);

    Serial.printf("armed=%d hovering=%d autopilot=%d\n"
            , armingButton.Read()
            , hoveringButton.Read()
            , autopilotButton.Read());
}
