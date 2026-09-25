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
#include <firmware/msp/__messages__.h>
#include <firmware/msp/serializer.hpp>
#include <firmware/pushbutton.hpp>
#include <firmware/voltage_divider.hpp>

// Hardware-dependent --------------------------------------------------------

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

// Analog input ---------------------------------------------------------------

static const uint8_t kThrottlePin = A4;
static const uint8_t kRollPin = A3;
static const uint8_t kPitchPin = A2;
static const uint8_t kYawPin = A1;

static const uint8_t kVoltageDividerPin = A6;

static auto armingButton = hf::Pushbutton(A8);
static auto autopilotButton = hf::Pushbutton(A7);
static auto hoveringButton = hf::Pushbutton(A0);

// ----------------------------------------------------------------------------

// Axis extrema determined empirically
static const uint16_t kThrottleLow = 3931;
static const uint16_t kThrottleHigh = 252;;
static const uint16_t kRollLow = 3093;
static const uint16_t kRollHigh = 766;
static const uint16_t kPitchLow = 217;
static const uint16_t kPitchHigh = 4038;
static const uint16_t kYawLow = 286;
static const uint16_t kYawHigh = 3736;

static const uint8_t kLedPin = 21;

static const float kVoltageDividerR1Ohms = 1000;
static const float kVoltageDividerR2Ohms = 2200;

static const float kLowVoltage = 3.0;

static const float kTransmitHz = 100;

static const uint8_t kLedIntensity = 255;

static auto blink_timer_ = hf::BlinkTimer();

static auto transmit_timer_ = hf::Timer(kTransmitHz);

static hf::VoltageDivider voltage_divider_ = hf::VoltageDivider(
        kVoltageDividerPin,
        kVoltageDividerR1Ohms,
        kVoltageDividerR2Ohms,
        12);

static auto ReadAxis(
        const uint8_t pin, const float lo, const float hi) -> float
{
    return (-analogRead(pin) + hi) / (-lo + hi) - 0.5;
}

static auto ReadAxis(const uint8_t pin) -> float
{
    return analogRead(pin);
}

static auto ReadAxisShort(
        const uint8_t pin, const short lo, const short hi) -> short
{  
    return map(analogRead(pin), hi, lo, 0, 4095);
}

static uint16_t low_, high_;

void setup()
{
    Serial.begin(115200);

    pinMode(kLedPin, OUTPUT);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);

    low_ = 1000;
    high_ = 0;
}

// Scale to [0, 4095]
static auto ReadAxis(
        const uint8_t pin, const uint16_t low, const uint16_t high) -> uint16_t
{
    return map(analogRead(pin), low, high, 0, 4095);
}

void loop()
{
    const auto volts = voltage_divider_.read();

    const auto ledState = volts < kLowVoltage ? blink_timer_.On() : true;

    analogWrite(kLedPin, ledState ? kLedIntensity : 0);

    const short vals[7] = {

        ReadAxis(kThrottlePin, kThrottleLow, kThrottleHigh),
        ReadAxis(kRollPin, kRollLow, kRollHigh),
        ReadAxis(kPitchPin, kPitchLow, kPitchHigh),
        ReadAxis(kYawPin, kYawLow, kYawHigh),

        armingButton.Read(),
        hoveringButton.Read(),
        autopilotButton.Read()
    };

    static hf::MspSerializer serializer_;

    serializer_ = hf::MspSerializer::SerializeShorts(
            serializer_, kMspSetChannels, vals, 7);

    esp_now_send(kReceiverAddress,
            hf::MspSerializer::GetPayloadBytes(serializer_),
            hf::MspSerializer::GetPayloadSize(serializer_));

    delay(10);
}
