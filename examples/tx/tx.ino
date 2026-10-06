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

static const uint8_t kReceiverAddress[6] = {
    
    // OMGS3
    0x98,0x3D,0xAE,0xEF,0x0E,0xAC
};


// Analog input ---------------------------------------------------------------

static const uint8_t kThrottlePin = A4;
static const uint8_t kRollPin = A3;
static const uint8_t kPitchPin = A2;
static const uint8_t kYawPin = A1;

static const uint8_t kVoltageDividerPin = A6;

static const uint8_t kArmingButtonPin = A8;
static const uint8_t kHoveringButtonPin = A0;
static const uint8_t kAutopilotButtonPin = A7;

static hf::Pushbutton arming_button_;
static hf::Pushbutton hovering_button_;
static hf::Pushbutton autopilot_button_;

// ----------------------------------------------------------------------------

// Axis extrema determined empirically
static const uint16_t kThrottleLow = 3931;
static const uint16_t kThrottleHigh = 252;;
static const uint16_t kRollMid = 2000;
static const uint16_t kPitchMid = 1950;
static const uint16_t kYawMid = 1900;

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

// Scale to [-2048, +2047]
static auto ReadAxis(const uint8_t pin, const int16_t mid) -> int16_t
{
    return analogRead(pin) - mid;
}

static auto DebugAxis(const uint8_t pin, const int16_t mid) -> float
{
    return ReadAxis(pin, mid) / 2048.f;
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

    const auto led_state = volts < kLowVoltage ? blink_timer_.On() : true;

    analogWrite(kLedPin, led_state ? kLedIntensity : 0);

    const auto msec = millis();

    const short vals[7] = {

        map(analogRead(kThrottlePin), kThrottleLow, kThrottleHigh, 0, 4095),
        -ReadAxis(kRollPin, kRollMid),
        ReadAxis(kPitchPin, kPitchMid),
        ReadAxis(kYawPin, kYawMid),

        arming_button_.Debounce(analogRead(kArmingButtonPin), msec),
        hovering_button_.Debounce(analogRead(kHoveringButtonPin), msec),
        autopilot_button_.Debounce(analogRead(kAutopilotButtonPin), msec)
    };

    printf("t=%+04d r=%+04d p=%+04d y=%+04d | arm=%d hov=%d aut=%d\n",
            vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]);

    static hf::MspSerializer serializer_;

    serializer_ = hf::MspSerializer::SerializeShorts(
            serializer_, kMspSetChannels, vals, 7);

    const auto result = esp_now_send(kReceiverAddress,
            hf::MspSerializer::GetPayloadBytes(serializer_),
            hf::MspSerializer::GetPayloadSize(serializer_));

    if (result != ESP_OK) {
        //Serial.printf("ERROR sending to vehicle: %d\n", result);
    }

    delay(10);
}
