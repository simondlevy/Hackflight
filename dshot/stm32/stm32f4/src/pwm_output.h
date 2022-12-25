/*
This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once


#include <time.h>

#include "platform.h"
#include "dma.h"
#include "io_types.h"
#include "motordev.h"
#include "timer.h"

#define BRUSHED_MOTORS_PWM_RATE 16000
#define BRUSHLESS_MOTORS_PWM_RATE 480

#define ALL_MOTORS 255

#define MOTOR_OUTPUT_LIMIT_PERCENT_MIN 1
#define MOTOR_OUTPUT_LIMIT_PERCENT_MAX 100

#define PWM_TIMER_1MHZ        MHZ_TO_HZ(1)

struct timerHardware_s;

typedef struct {
    volatile timCCR_t *ccr;
    TIM_TypeDef       *tim;
} timerChannel_t;

typedef struct {
    timerChannel_t channel;
    float pulseScale;
    float pulseOffset;
    bool forceOverflow;
    bool enabled;
    IO_t io;
} pwmOutputPort_t;

extern FAST_DATA_ZERO_INIT pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];

struct motorDevConfig_s;
motorDevice_t *motorPwmDevInit(uint16_t idlePulse, uint8_t motorCount, bool useUnsyncedPwm);

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion);

pwmOutputPort_t *pwmGetMotors(void);
bool pwmIsSynced(void);
