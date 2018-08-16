/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"
#include "drivers/time.h"

#include "drivers/io.h"
#include "pwm_output.h"
#include "timer.h"
#include "drivers/pwm_output.h"

static FAST_RAM_ZERO_INIT pwmWriteFn *pwmWrite;
static FAST_RAM_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static FAST_RAM_ZERO_INIT pwmCompleteWriteFn *pwmCompleteWrite = NULL;

FAST_RAM_ZERO_INIT loadDmaBufferFn *loadDmaBuffer;
#define DSHOT_INITIAL_DELAY_US 10000
#define DSHOT_COMMAND_DELAY_US 1000
#define DSHOT_ESCINFO_DELAY_US 12000
#define DSHOT_BEEP_DELAY_US 100000

typedef struct dshotCommandControl_s {
    timeUs_t nextCommandAtUs;
    timeUs_t delayAfterCommandUs;
    bool waitingForIdle;
    uint8_t repeats;
    uint8_t command[MAX_SUPPORTED_MOTORS];
} dshotCommandControl_t;

static dshotCommandControl_t dshotCommandControl;

static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];


static bool pwmMotorsEnabled = false;
static bool isDshot = false;
FAST_RAM_ZERO_INIT bool useBurstDshot = false;

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value, uint8_t output)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

    if (output & TIMER_OUTPUT_N_CHANNEL) {
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
        TIM_OCInitStructure.TIM_OCNPolarity = (output & TIMER_OUTPUT_INVERTED) ? TIM_OCNPolarity_Low : TIM_OCNPolarity_High;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCPolarity =  (output & TIMER_OUTPUT_INVERTED) ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
    }
    TIM_OCInitStructure.TIM_Pulse = value;

    timerOCInit(tim, channel, &TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, TIM_OCPreload_Enable);
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    configTimeBase(timerHardware->tim, period, hz);
    pwmOCConfig(timerHardware->tim,
        timerHardware->channel,
        value,
        inversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output
        );

    TIM_CtrlPWMOutputs(timerHardware->tim, ENABLE);
    TIM_Cmd(timerHardware->tim, ENABLE);

    channel->ccr = timerChCCR(timerHardware);

    channel->tim = timerHardware->tim;

    *channel->ccr = 0;
}

static void pwmWriteUnused(uint8_t index, float value)
{
    UNUSED(index);
    UNUSED(value);
}

static void pwmWriteStandard(uint8_t index, float value)
{
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    *motors[index].channel.ccr = lrintf((value * motors[index].pulseScale) + motors[index].pulseOffset);
}

void pwmWriteMotor(uint8_t index, float value)
{
    pwmWrite(index, value);
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if (motors[index].channel.ccr) {
            *motors[index].channel.ccr = 0;
        }
    }
}

void pwmDisableMotors(void)
{
    pwmShutdownPulsesForAllMotors(MAX_SUPPORTED_MOTORS);
    pwmMotorsEnabled = false;
}

void pwmEnableMotors(void)
{
    /* check motors can be enabled */
    pwmMotorsEnabled = (pwmWrite != &pwmWriteUnused);
}

bool pwmAreMotorsEnabled(void)
{
    return pwmMotorsEnabled;
}

static void pwmCompleteWriteUnused(uint8_t motorCount)
{
    UNUSED(motorCount);
}

static void pwmCompleteOneshotMotorUpdate(uint8_t motorCount)
{
    for (int index = 0; index < motorCount; index++) {
        if (motors[index].forceOverflow) {
            timerForceOverflow(motors[index].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        *motors[index].channel.ccr = 0;
    }
}

void pwmCompleteMotorUpdate(uint8_t motorCount)
{
    pwmCompleteWrite(motorCount);
}

void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t idlePulse, uint8_t motorCount)
{
    memset(motors, 0, sizeof(motors));

    bool useUnsyncedPwm = motorConfig->useUnsyncedPwm;

    float sMin = 0;
    float sLen = 0;
    switch (motorConfig->motorPwmProtocol) {
    case PWM_TYPE_BRUSHED:
        sMin = 0;
        useUnsyncedPwm = true;
        idlePulse = 0;
        break;
    default:
        sMin = 1e-3f;
        sLen = 1e-3f;
        useUnsyncedPwm = true;
        idlePulse = 0;
    }

    if (!isDshot) {
        pwmWrite = &pwmWriteStandard;
        pwmCompleteWrite = useUnsyncedPwm ? &pwmCompleteWriteUnused : &pwmCompleteOneshotMotorUpdate;
    }

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        const ioTag_t tag = motorConfig->ioTags[motorIndex];
        const timerHardware_t *timerHardware = timerGetByTag(tag);

        if (timerHardware == NULL) {
            /* not enough motors initialised for the mixer or a break in the motors */
            pwmWrite = &pwmWriteUnused;
            pwmCompleteWrite = &pwmCompleteWriteUnused;
            /* TODO: block arming and add reason system cannot arm */
            return;
        }

        motors[motorIndex].io = IOGetByTag(tag);
        IOInit(motors[motorIndex].io, OWNER_MOTOR, RESOURCE_INDEX(motorIndex));

        if (isDshot) {
            pwmDshotMotorHardwareConfig(timerHardware,
                    motorIndex,
                    motorConfig->motorPwmProtocol,
                    motorConfig->motorPwmInversion ? timerHardware->output ^ TIMER_OUTPUT_INVERTED : timerHardware->output);
            motors[motorIndex].enabled = true;
            continue;
        }

        IOConfigGPIOAF(motors[motorIndex].io, IOCFG_AF_PP, timerHardware->alternateFunction);

        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        const unsigned pwmRateHz = useUnsyncedPwm ? motorConfig->motorPwmRate : ceilf(1 / ((sMin + sLen) * 4));

        const uint32_t clock = timerClock(timerHardware->tim);
        /* used to find the desired timer frequency for max resolution */
        const unsigned prescaler = ((clock / pwmRateHz) + 0xffff) / 0x10000; /* rounding up */
        const uint32_t hz = clock / prescaler;
        const unsigned period = useUnsyncedPwm ? hz / pwmRateHz : 0xffff;

        /*
           if brushed then it is the entire length of the period.
TODO: this can be moved back to periodMin and periodLen
once mixer outputs a 0..1 float value.
         */
        motors[motorIndex].pulseScale = ((motorConfig->motorPwmProtocol == PWM_TYPE_BRUSHED) ? period : (sLen * hz)) / 1000.0f;
        motors[motorIndex].pulseOffset = (sMin * hz) - (motors[motorIndex].pulseScale * 1000);

        pwmOutConfig(&motors[motorIndex].channel, timerHardware, hz, period, idlePulse, motorConfig->motorPwmInversion);

        bool timerAlreadyUsed = false;
        for (int i = 0; i < motorIndex; i++) {
            if (motors[i].channel.tim == motors[motorIndex].channel.tim) {
                timerAlreadyUsed = true;
                break;
            }
        }
        motors[motorIndex].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex].enabled = true;
    }

    pwmMotorsEnabled = true;
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

bool isMotorProtocolDshot(void)
{
    return isDshot;
}

uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType)
{
    switch (pwmProtocolType) {
        case(PWM_TYPE_PROSHOT1000):
            return MOTOR_PROSHOT1000_HZ;
        case(PWM_TYPE_DSHOT1200):
            return MOTOR_DSHOT1200_HZ;
        case(PWM_TYPE_DSHOT600):
            return MOTOR_DSHOT600_HZ;
        case(PWM_TYPE_DSHOT300):
            return MOTOR_DSHOT300_HZ;
        default:
        case(PWM_TYPE_DSHOT150):
            return MOTOR_DSHOT150_HZ;
    }
}

bool allMotorsAreIdle(uint8_t motorCount)
{
    bool allMotorsIdle = true;
    for (unsigned i = 0; i < motorCount; i++) {
        const motorDmaOutput_t *motor = getMotorDmaOutput(i);
        if (motor->value) {
            allMotorsIdle = false;
        }
    }

    return allMotorsIdle;
}

FAST_CODE bool pwmDshotCommandIsQueued(void)
{
    return dshotCommandControl.nextCommandAtUs;
}

FAST_CODE bool pwmDshotCommandIsProcessing(void)
{
    return dshotCommandControl.nextCommandAtUs && !dshotCommandControl.waitingForIdle && dshotCommandControl.repeats > 0;
}

void pwmWriteDshotCommand(uint8_t index, uint8_t motorCount, uint8_t command, bool blocking)
{
    timeUs_t timeNowUs = micros();

    if (!isMotorProtocolDshot() || (command > DSHOT_MAX_COMMAND) || pwmDshotCommandIsQueued()) {
        return;
    }

    uint8_t repeats = 1;
    timeUs_t delayAfterCommandUs = DSHOT_COMMAND_DELAY_US;

    switch (command) {
    case DSHOT_CMD_SPIN_DIRECTION_1:
    case DSHOT_CMD_SPIN_DIRECTION_2:
    case DSHOT_CMD_3D_MODE_OFF:
    case DSHOT_CMD_3D_MODE_ON:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        repeats = 10;
        break;
    case DSHOT_CMD_BEACON1:
    case DSHOT_CMD_BEACON2:
    case DSHOT_CMD_BEACON3:
    case DSHOT_CMD_BEACON4:
    case DSHOT_CMD_BEACON5:
        delayAfterCommandUs = DSHOT_BEEP_DELAY_US;
        break;
    default:
        break;
    }

    if (blocking) {
        delayMicroseconds(DSHOT_INITIAL_DELAY_US - DSHOT_COMMAND_DELAY_US);
        for (; repeats; repeats--) {
            delayMicroseconds(DSHOT_COMMAND_DELAY_US);

            for (uint8_t i = 0; i < motorCount; i++) {
                if ((i == index) || (index == ALL_MOTORS)) {
                    motorDmaOutput_t *const motor = getMotorDmaOutput(i);
                    motor->requestTelemetry = true;
                    pwmWriteDshotInt(i, command);
                }
            }

            pwmCompleteDshotMotorUpdate(0);
        }
        delayMicroseconds(delayAfterCommandUs);
    } else {
        dshotCommandControl.repeats = repeats;
        dshotCommandControl.nextCommandAtUs = timeNowUs + DSHOT_INITIAL_DELAY_US;
        dshotCommandControl.delayAfterCommandUs = delayAfterCommandUs;
        for (unsigned i = 0; i < motorCount; i++) {
            if (index == i || index == ALL_MOTORS) {
                dshotCommandControl.command[i] = command;
            } else {
                dshotCommandControl.command[i] = command;
            }
        }

        dshotCommandControl.waitingForIdle = !allMotorsAreIdle(motorCount);
    }
}

uint8_t pwmGetDshotCommand(uint8_t index)
{
    return dshotCommandControl.command[index];
}

FAST_CODE_NOINLINE bool pwmDshotCommandOutputIsEnabled(uint8_t motorCount)
{
    timeUs_t timeNowUs = micros();

    if (dshotCommandControl.waitingForIdle) {
        if (allMotorsAreIdle(motorCount)) {
            dshotCommandControl.nextCommandAtUs = timeNowUs + DSHOT_INITIAL_DELAY_US;
            dshotCommandControl.waitingForIdle = false;
        }

        // Send normal motor output while waiting for motors to go idle
        return true;
    }

    if (cmpTimeUs(timeNowUs, dshotCommandControl.nextCommandAtUs) < 0) {
        //Skip motor update because it isn't time yet for a new command
        return false;
    }   
  
    //Timed motor update happening with dshot command
    if (dshotCommandControl.repeats > 0) {
        dshotCommandControl.repeats--;

        if (dshotCommandControl.repeats > 0) {
            dshotCommandControl.nextCommandAtUs = timeNowUs + DSHOT_COMMAND_DELAY_US;
        } else {
            dshotCommandControl.nextCommandAtUs = timeNowUs + dshotCommandControl.delayAfterCommandUs;
        }
    } else {
        dshotCommandControl.nextCommandAtUs = 0;
    }

    return true;
}

FAST_CODE uint16_t prepareDshotPacket(motorDmaOutput_t *const motor)
{
    uint16_t packet = (motor->value << 1) | (motor->requestTelemetry ? 1 : 0);
    motor->requestTelemetry = false;    // reset telemetry request to make sure it's triggered only once in a row

    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;

    return packet;
}

void pwmWriteServo(uint8_t index, float value)
{
    if (index < MAX_SUPPORTED_SERVOS && servos[index].channel.ccr) {
        *servos[index].channel.ccr = lrintf(value);
    }
}

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++) {
        const ioTag_t tag = servoConfig->ioTags[servoIndex];

        if (!tag) {
            break;
        }

        servos[servoIndex].io = IOGetByTag(tag);

        IOInit(servos[servoIndex].io, OWNER_SERVO, RESOURCE_INDEX(servoIndex));

        const timerHardware_t *timer = timerGetByTag(tag);
        IOConfigGPIO(servos[servoIndex].io, IOCFG_AF_PP);

        if (timer == NULL) {
            /* flag failure and disable ability to arm */
            break;
        }
        pwmOutConfig(&servos[servoIndex].channel, timer, PWM_TIMER_1MHZ, PWM_TIMER_1MHZ / servoConfig->servoPwmRate, servoConfig->servoCenterPulse, 0);
        servos[servoIndex].enabled = true;
    }
}
