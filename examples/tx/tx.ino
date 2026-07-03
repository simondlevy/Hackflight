/* Hackflight ESP32 transmitter sketch

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

static const uint8_t THROTTLE_INPUT_PIN = 4;
static const uint8_t ROLL_INPUT_PIN = 14;
static const uint8_t PITCH_INPUT_PIN = 15;
static const uint8_t YAW_INPUT_PIN = 27;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.printf("t=%d r=%d p=%d y=%d\n",
            analogRead(THROTTLE_INPUT_PIN),
            analogRead(ROLL_INPUT_PIN),
            analogRead(PITCH_INPUT_PIN),
            analogRead(YAW_INPUT_PIN));
}
