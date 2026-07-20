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

#include "pushbutton.hpp"

static const uint8_t THROTTLE_PIN = 25;
static const uint8_t ROLL_PIN = 33;
static const uint8_t PITCH_PIN = 32;
static const uint8_t YAW_PIN = 4;

static const uint8_t ARMING_PIN = 23;
static const uint8_t HOVER_PIN = 27;
static const uint8_t AUTOPILOT_PIN = 19;

static const uint8_t VOLTAGE_DIVIDER_PIN = 14;
static const float VOLTAGE_DIVIDER_R1_OHMS = 1000;
static const float VOLTAGE_DIVIDER_R2_OHMS = 2200;

static const uint8_t LED_PIN = 15;

static const float ANALOG_MIN = 240;
static const float ANALOG_MAX = 3900;

static const float LOW_VOLTAGE = 3.0;
static const float LED_BLINK_HZ = 2;

static bool arming_prev_;

static PushButton hoverButton_ = PushButton(HOVER_PIN);;
static PushButton autopilotButton_ = PushButton(AUTOPILOT_PIN);;

static hf::VoltageDivider voltage_divider_ = hf::VoltageDivider(
        VOLTAGE_DIVIDER_PIN,
        VOLTAGE_DIVIDER_R1_OHMS,
        VOLTAGE_DIVIDER_R2_OHMS,
        12);

static auto ReadArmingSwitch() -> bool
{
    return digitalRead(ARMING_PIN);
}

static auto ReadGimbal(const uint8_t pin) -> float
{
    return (analogRead(pin) - ANALOG_MIN) / (ANALOG_MAX - ANALOG_MIN);
}

static void blinkLeds()
{
    static uint32_t msec_;
    static bool on_;

    const auto msec = millis();
    
    if (msec - msec_ > 1000/LED_BLINK_HZ) {
        digitalWrite(LED_PIN, on_);
        msec_ = msec;
        on_ = !on_;
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(ARMING_PIN, INPUT);
    pinMode(HOVER_PIN, INPUT);
    pinMode(AUTOPILOT_PIN, INPUT);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    hoverButton_.begin();
    autopilotButton_.begin();

    arming_prev_ = ReadArmingSwitch();
}

void loop()
{
    static bool armed_;
    const auto arming_curr = ReadArmingSwitch();
    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }
    arming_prev_ = arming_curr;

    const auto hovering = hoverButton_.read();

    const auto autopilot = autopilotButton_.read();

    const auto throttle = 1 - ReadGimbal(THROTTLE_PIN);

    const auto roll = 2 * (0.5 - ReadGimbal(ROLL_PIN));

    const auto pitch = 2 * (ReadGimbal(PITCH_PIN) - 0.5);

    const auto yaw = 2 * ReadGimbal(YAW_PIN) - 1;

    const auto volts = voltage_divider_.read();

    if (volts < LOW_VOLTAGE) {
        blinkLeds();
    }

    Serial.printf("throttle=%3.2f roll=%+3.2f pitch=%+3.2f yaw=%+3.2f | "
            "armed=%d hovering=%d autopilot=%d | voltage=%3.3f\n",
            throttle, roll, pitch, yaw, armed_, hovering, autopilot, volts);
}
