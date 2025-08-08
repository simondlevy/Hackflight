/*
   Test ESCs

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <oneshot125.hpp>
#include <vector>

static const std::vector<uint8_t> PINS = {2, 23, 14, 9};

static float MAX = 0.5;
static float INC = 1e-5;

static auto motors = OneShot125(PINS);

static float val;
static float dir;

static volatile bool running;

void serialEvent()
{
  while (Serial.available()) {
    Serial.read();
  }

  running = !running;
}

void setup() 
{
    Serial.begin(115200);

    motors.arm(); 

    dir = +1;

}

static float inputGet()
{
    if (running) {

        val += INC * dir;

        if (val >= MAX) {
            dir = -1;
        }

        if (val <= 0) {
            dir = +1;
        }
    }
    else {
        val = 0;
    }

    return val;
}

static void prompt()
{
    static uint32_t tprev;
    const uint32_t tcurr = millis();
    if (tcurr - tprev > 1000) {
      tprev = tcurr;
      Serial.print("Hit Enter to ");
      Serial.println(running ? "stop" : "start");
    }
}

void loop() 
{
    const float input = inputGet();

    auto pulseWidth = (uint8_t)(125 * (input + 1));

    motors.set(0, pulseWidth);
    motors.set(1, pulseWidth);
    motors.set(2, pulseWidth);
    motors.set(3, pulseWidth);

    motors.run();

    prompt();
}
