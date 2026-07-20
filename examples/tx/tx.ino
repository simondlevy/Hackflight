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
#include <firmware/voltage_divider.hpp>
#include <firmware/analog_pushbutton.hpp>

static const uint8_t kThrottlePin = 25;
static const uint8_t kRollPin = 33;
static const uint8_t kPitchPin = 32;
static const uint8_t kYawPin = 4;

static const uint8_t kArmingPin = 23;
static const uint8_t kHoverPin = 15;

static const uint8_t kVoltageDividerPin = 14;
static const float kVoltageDividerR1Ohms = 1000;
static const float kVoltageDividerR2Ohms = 2200;

static const uint8_t KLedPin = 15;

static const float kAnalogMin = 240;
static const float kAnalogMax = 3900;

static const float kLowVoltage = 3.0;
static const float kLedBlinkHz = 2;

static const uint16_t kAnalogThreshold = 4000;

static bool arming_prev_;

static hf::VoltageDivider voltage_divider_ = hf::VoltageDivider(
        kVoltageDividerPin,
        kVoltageDividerR1Ohms,
        kVoltageDividerR2Ohms,
        12);

static auto ReadGimbal(const uint8_t pin) -> float
{
    return (analogRead(pin) - kAnalogMin) / (kAnalogMax - kAnalogMin);
}

static auto AnalogThreshold(const uint8_t pin) -> bool
{
    return analogRead(pin) > kAnalogThreshold;
}

static AnalogPushButton hoverButton_ = AnalogPushButton(kHoverPin);

static void blinkLeds()
{
    static uint32_t msec_;
    static bool on_;

    const auto msec = millis();
    
    if (msec - msec_ > 1000/kLedBlinkHz) {
        digitalWrite(KLedPin, on_);
        msec_ = msec;
        on_ = !on_;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(KLedPin, OUTPUT);
    digitalWrite(KLedPin, HIGH);

    arming_prev_ = AnalogThreshold(kArmingPin);
}

void loop()
{
    static bool armed_;
    const auto arming_curr = AnalogThreshold(kArmingPin);
    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }
    arming_prev_ = arming_curr;

    const auto hovering = hoverButton_.Read();

    const auto throttle = 1 - ReadGimbal(kThrottlePin);

    const auto roll = 2 * (0.5 - ReadGimbal(kRollPin));

    const auto pitch = 2 * (ReadGimbal(kPitchPin) - 0.5);

    const auto yaw = 2 * ReadGimbal(kYawPin) - 1;

    const auto volts = voltage_divider_.read();

    if (volts < kLowVoltage) {
        blinkLeds();
    }

    Serial.printf("throttle=%3.2f roll=%+3.2f pitch=%+3.2f yaw=%+3.2f | "
            "armed=%d hovering=%d | voltage=%3.3f\n",
            throttle, roll, pitch, yaw, armed_, hovering, volts);
}
