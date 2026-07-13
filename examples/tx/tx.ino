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

#include "pushbutton.hpp"

static const uint8_t THROTTLE_INPUT_PIN = 25;
static const uint8_t ROLL_INPUT_PIN = 33;
static const uint8_t PITCH_INPUT_PIN = 32;
static const uint8_t YAW_INPUT_PIN = 4;

//static const uint8_t ARMING_INPUT_PIN = 26;
//static const uint8_t HOVER_INPUT_PIN = 25;
//static const uint8_t AUTOPILOT_INPUT_PIN = 23;

//static const uint8_t LED_PIN = 15;

static const float ANALOG_MIN = 240;
static const float ANALOG_MAX = 3900;

static bool arming_prev_;

//static PushButton hoverButton_ = PushButton(HOVER_INPUT_PIN);;
//static PushButton autopilotButton_ = PushButton(AUTOPILOT_INPUT_PIN);;

/*
static auto ReadArmingSwitch() -> bool
{
    return digitalRead(ARMING_INPUT_PIN);
}*/

static auto ReadGimbal(const uint8_t pin) -> float
{
    return (analogRead(pin) - ANALOG_MIN) / (ANALOG_MAX - ANALOG_MIN);
}

void setup()
{
    Serial.begin(115200);

    //pinMode(ARMING_INPUT_PIN, INPUT);
    //pinMode(HOVER_INPUT_PIN, INPUT);
    //pinMode(AUTOPILOT_INPUT_PIN, INPUT);

    //pinMode(LED_PIN, OUTPUT);

    //hoverButton_.begin();
    //autopilotButton_.begin();

    //arming_prev_ = ReadArmingSwitch();
}

void loop()
{
    static bool armed_;
    /*
    const auto arming_curr = ReadArmingSwitch();
    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }
    arming_prev_ = arming_curr;
    */

    const auto hovering = false;//hoverButton_.read();

    const auto autopilot = false;//autopilotButton_.read();

    const auto throttle = 1 - ReadGimbal(THROTTLE_INPUT_PIN);

    const auto roll = 2 * (0.5 - ReadGimbal(ROLL_INPUT_PIN));

    const auto pitch = 2 * (ReadGimbal(PITCH_INPUT_PIN) - 0.5);

    const auto yaw = 2 * ReadGimbal(YAW_INPUT_PIN) - 1;

    Serial.printf("throttle=%3.2f roll=%+3.2f pitch=%+3.2f yaw=%+3.2f | "
            "armed=%d hovering=%d autopilot=%d\n",
            throttle, roll, pitch, yaw, armed_, hovering, autopilot);
}
