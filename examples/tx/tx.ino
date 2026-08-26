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
#include <firmware/pushbuttons.hpp>
#include <firmware/voltage_divider.hpp>

// Hardware-dependent --------------------------------------------------------

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const short kThrottleLo = 280;
static const short kThrottleHi = 3800;

static const short kRollLo = 750;
static const short kRollHi = 3050;
static const short kRollMid = 1980;

static const short kPitchLo = 3980;
static const short kPitchHi = 250;

static const short kYawLo = 3680;
static const short kYawHi = 320;

static const uint8_t kLedIntensity = 255;

// Analog input ---------------------------------------------------------------

static const uint8_t kThrottlePin = A4;
static const uint8_t kRollPin = A3;
static const uint8_t kPitchPin = A2;
static const uint8_t kYawPin = A1;

static const uint8_t kVoltageDividerPin = A6;

static auto armingButton = hf::LatchingPushbutton(A8);

static auto autopilotButton = hf::IntermittentPushbutton(A7);
static auto hoveringButton = hf::IntermittentPushbutton(A0);

// ----------------------------------------------------------------------------

static const uint8_t kLedPin = 21;

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

    const auto ledState = volts < kLowVoltage ? blink_timer_.On() : true;

    analogWrite(kLedPin, ledState ? kLedIntensity : 0);

    Serial.printf("rol=%04d\n", kRollMid - analogRead(kRollPin));

    const short vals[7] = {
            analogRead(kThrottlePin),
            analogRead(kRollPin),
            analogRead(kPitchPin),
            analogRead(kYawPin),
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
