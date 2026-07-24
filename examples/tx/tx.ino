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
#include <firmware/blink_timer.hpp>
#include <firmware/espnow.hpp>
#include <firmware/voltage_divider.hpp>
#include <firmware/analog_pushbutton.hpp>

static const uint8_t kReceiverAddress[6] = {0x98,0x3D,0xAE,0xEF,0x0E,0xAC};

static const bool kDebug = true;

static const uint8_t kYawPin = 4;
static const uint8_t kVoltageDividerPin = 14;
static const uint8_t kLedPin = 15;
static const uint8_t kArmingPin = 23;
static const uint8_t kThrottlePin = 25;
static const uint8_t kHoverPin = 27;
static const uint8_t kPitchPin = 32;
static const uint8_t kRollPin = 33;

static const float kVoltageDividerR1Ohms = 1000;
static const float kVoltageDividerR2Ohms = 2200;

static const float kAnalogMin = 240;
static const float kAnalogMax = 3900;

static const float kLowVoltage = 3.0;

static const float kTransmitHz = 100;

static const uint16_t kAnalogPushbuttonThreshold = 4094;

static auto blink_timer_ = hf::BlinkTimer();

static auto transmit_timer_ = hf::Timer(kTransmitHz);

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

static AnalogPushButton hoverButton_ = AnalogPushButton(kHoverPin,
        kAnalogPushbuttonThreshold);

void setup()
{
    Serial.begin(115200);

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, HIGH);

    pinMode(kArmingPin, INPUT);

    arming_prev_ = digitalRead(kArmingPin);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kReceiverAddress);
}

void loop()
{
    static bool armed_;
    const auto arming_curr = digitalRead(kArmingPin);
    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }
    arming_prev_ = arming_curr;

    // const auto hovering = hoverButton_.Read();
    // const auto throttle = 1 - ReadGimbal(kThrottlePin);
    // const auto roll = 2 * (0.5 - ReadGimbal(kRollPin));
    // const auto pitch = 2 * (ReadGimbal(kPitchPin) - 0.5);
    // const auto yaw = 2 * ReadGimbal(kYawPin) - 1;
    const auto volts = 0; //voltage_divider_.read();

    if (volts < kLowVoltage) {
        digitalWrite(kLedPin, blink_timer_.On());
    }

    //if (kDebug) {
    //    Serial.printf("throttle=%3.2f roll=%+3.2f pitch=%+3.2f yaw=%+3.2f | "
    //            "armed=%d hovering=%d | voltage=%3.3f\n",
    //            throttle, roll, pitch, yaw, armed_, hovering, volts);
    //}

    const uint8_t data = 'A';

    if (transmit_timer_.Ready()) {
        if (esp_now_send(kReceiverAddress, &data, 1) != ESP_OK) {
            Serial.println("Error sending the data");
        }
    }
}
