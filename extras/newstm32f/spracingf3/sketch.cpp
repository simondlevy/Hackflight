#include <hackflight.hpp>

extern "C" {

// Cleanflight includes
#include "platform.h"
#include "drivers/io.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"

    static const uint16_t BRUSHED_MOTORS_PWM_RATE   = 16000;
    static const uint16_t BRUSHLESS_MOTORS_PWM_RATE = 480;

    typedef enum {
        MOTOR_UNKNOWN = 0,
        MOTOR_BRUSHED,
        MOTOR_BRUSHLESS
    } HardwareMotorTypes_e;

    static float motorval;

    void setup(void)
    {
        motorval = .01;
    }

    void loop(void)
    {
        pwmWriteMotor(0, motorval);
        motorval += .01;
        delay(10);
    }

} // extern "C"
