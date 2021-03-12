/*
   Arduino sketch to test brushless motor with standard ESC

   DID YOU REMEMOVE THE PROPELLERS FIRST?

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include "hackflight.hpp"
#include "motors/standard.hpp"

static uint8_t MOTOR_PIN[1] = {15};

static float  val;
static int8_t dir;

hf::StandardMotor motors = hf::StandardMotor(MOTOR_PIN, 1);

void setup(void)
{
    // Initialize the motor
    motors.init();

    // Start with motor off, increasing
    val = 0;
    dir = +1;

    delay(1000);
}

void loop(void)
{
    motors.write(0, val);

    val += dir * .001;

    // stop halfway
    if (val >= 0.5) {
        dir = -1;
    }

    if (val <= 0) {
        dir = +1;
    }

    delay(10);
}
