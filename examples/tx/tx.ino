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

static const uint8_t kThrottlePin = A5;
static const uint8_t kRollPin = A2;
static const uint8_t kPitchPin = A3;
static const uint8_t kYawPin = A6;

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

static auto ReadAxis(
        const uint8_t pin, const float lo, const float hi) -> float
{
    return (-analogRead(pin) - lo) / (hi - lo) - 0.5;
}

static auto ReadAxis(const uint8_t pin) -> float
{
    return analogRead(pin);
}

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

    Serial.printf(
            "thr=%+5.3f rol=%+5.3f | "
            "armed=%d hovering=%d autopilot=%d\n"
            , ReadAxis(kThrottlePin, -3800, -280)
            , ReadAxis(kRollPin, -3050, -740)
            //, ReadAxis(kRollPin)
            , armingButton.Read()
            , hoveringButton.Read()
            , autopilotButton.Read());
}
