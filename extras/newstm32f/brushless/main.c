#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"

// ======================================================

#include "drivers/pwm_output.h"

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 480

typedef enum {
    MOTOR_UNKNOWN = 0,
    MOTOR_BRUSHED,
    MOTOR_BRUSHLESS
} HardwareMotorTypes_e;

uint8_t hardwareMotorType = MOTOR_BRUSHLESS;

static float motorval;

void setup(void)
{
    systemInit();

    IOInitGlobal();

    delay(100);

    timerInit();  

    uint16_t idlePulse = 1000; // 0 for brushed motor

    motorDevConfig_t dev;

    dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
    dev.motorPwmProtocol = PWM_TYPE_STANDARD;
    dev.motorPwmInversion = false;
    dev.useUnsyncedPwm = true;
    dev.useBurstDshot = false;

    dev.ioTags[0] = timerioTagGetByUsage(TIM_USE_MOTOR, 0);
    dev.ioTags[1] = timerioTagGetByUsage(TIM_USE_MOTOR, 1);
    dev.ioTags[2] = timerioTagGetByUsage(TIM_USE_MOTOR, 2);
    dev.ioTags[3] = timerioTagGetByUsage(TIM_USE_MOTOR, 3);

    motorDevInit(&dev, idlePulse, 4);

    pwmEnableMotors();

    motorval = .01;
}

void loop(void)
{
    pwmWriteMotor(0, motorval);
    motorval += .01;
    delay(10);
}

int main(void)
{
    void delay(unsigned long);

    setup();

    while (true) {

        loop();

    }
}
