#include "teensy_dshot.hpp"

static const uint8_t NMOTORS = 1;
static uint8_t motorList[NMOTORS] = { 22 };

static DshotMotor motor(NMOTORS, motorList);

void setup() 
{
    if (!motor.armed) {
        motor.armMotorESC();
    }
}

void loop() 
{

    motor.setRunSpeed(1000);

    delayMicroseconds(225);
}
