/* Hackflight ESP32 transmitter sketch Copyright (C) 2026 Simon D. Levy This
 * program is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, in version 3.  This program is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.  You should have received a copy of the GNU
 * General Public License
 along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

static const uint8_t THROTTLE_INPUT_PIN = 4;
static const uint8_t ROLL_INPUT_PIN = 14;
static const uint8_t PITCH_INPUT_PIN = 15;
static const uint8_t YAW_INPUT_PIN = 27;

static const uint8_t ARMING_INPUT_PIN = 26;

static const float ANALOG_MIN = 240;
static const float ANALOG_MAX = 3900;

static bool arming_prev_;

static auto ReadArmingSwitch() -> bool
{
    return digitalRead(ARMING_INPUT_PIN);
}

static auto ReadGimbal(const uint8_t pin) -> float
{
    return (analogRead(pin) - ANALOG_MIN) / (ANALOG_MAX - ANALOG_MIN);
}

void setup()
{
    Serial.begin(115200);

    pinMode(ARMING_INPUT_PIN, INPUT);

    arming_prev_ = ReadArmingSwitch();
}

void loop()
{
    const auto arming_curr = ReadArmingSwitch();

    static bool armed_;

    if (arming_prev_ != arming_curr) {
        armed_ = !armed_;
    }

    arming_prev_ = arming_curr;

    const auto throttle = 1 - ReadGimbal(THROTTLE_INPUT_PIN);

    const auto roll = 2 * (0.5 - ReadGimbal(ROLL_INPUT_PIN));

    const auto pitch = 2 * (ReadGimbal(PITCH_INPUT_PIN) - 0.5);

    const auto yaw = 2 * ReadGimbal(YAW_INPUT_PIN) - 1;

    Serial.printf("t=%3.2f r=%+3.2f p=%+3.2f y=%+3.2f | a=%d\n",
            throttle, roll, pitch, yaw, armed_);
}
