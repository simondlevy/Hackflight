/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ledseq.c - LED sequence handler
 */

#include <stdbool.h>

#include <free_rtos.h>
#include <semphr.h>
#include <timers.h>

#include "../../../build/include/generated/autoconf.h"

#include <led.h>
#include <ledseq.h>
#include <static_mem.h>

#include "ledseq.h"

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500

#define LEDSEQCMD_TASK_PRI      1
#define LEDSEQCMD_TASK_NAME     "LEDSEQCMD"
#define LEDSEQCMD_TASK_STACKSIZE      configMINIMAL_STACK_SIZE

//Led sequence action
#define LEDSEQ_WAITMS(X) (X)
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2

typedef struct {
  bool value;
  int action;
} ledseqStep_t;

typedef struct ledseqContext_s {
  ledseqStep_t* const sequence;
  struct ledseqContext_s* nextContext;
  int state;
  const uint8_t led;
} ledseqContext_t;


bool ledseqRun(ledseqContext_t* context);
void ledseqRunBlocking(ledseqContext_t* context);
bool ledseqStop(ledseqContext_t* context);
void ledseqStopBlocking(ledseqContext_t* context);
void ledseqSetChargeLevel(const float chargeLevel);

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500

/* Led sequences */
ledseqStep_t seq_lowbat_def[] = {
    { true, LEDSEQ_WAITMS(1000)},
    {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_lowbat = {
    .sequence = seq_lowbat_def,
    .led = LOWBAT_LED,
};

ledseqContext_t* sequences = NULL;

ledseqStep_t seq_calibrated_def[] = {
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(450)},
    {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_calibrated = {
    .sequence = seq_calibrated_def,
    .led = SYS_LED,
};

ledseqStep_t seq_alive_def[] = {
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(1950)},
    {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_alive = {
    .sequence = seq_alive_def,
    .led = SYS_LED,
};

ledseqStep_t seq_linkup_def[] = {
    { true, LEDSEQ_WAITMS(1)},
    {false, LEDSEQ_WAITMS(0)},
    {    0, LEDSEQ_STOP},
};

ledseqContext_t seq_linkUp = {
    .sequence = seq_linkup_def,
    .led = LINK_LED,
};

ledseqContext_t seq_linkDown = {
    .sequence = seq_linkup_def,
    .led = LINK_DOWN_LED,
};

ledseqStep_t seq_charged_def[] = {
    { true, LEDSEQ_WAITMS(1000)},
    {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_charged = {
    .sequence = seq_charged_def,
    .led = CHG_LED,
};

ledseqStep_t seq_charging_def[] = {
    { true, LEDSEQ_WAITMS(200)},
    {false, LEDSEQ_WAITMS(800)},
    {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_charging = {
    .sequence = seq_charging_def,
    .led = CHG_LED,
};

ledseqStep_t seq_testPassed_def[] = {
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    { true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_STOP},
};

ledseqContext_t seq_testPassed = {
    .sequence = seq_testPassed_def,
    .led = LINK_LED,
};

ledseqContext_t seq_testFailed = {
    .sequence = seq_testPassed_def,
    .led = SYS_LED,
};

typedef enum {
    run,
    stop
} command_t;

struct ledseqCmd_s {
    command_t command;
    ledseqContext_t *sequence;
};

NO_DMA_CCM_SAFE_ZERO_INIT static ledseqContext_t* activeSeq[LED_COUNT];
NO_DMA_CCM_SAFE_ZERO_INIT static xTimerHandle timer[LED_COUNT];
NO_DMA_CCM_SAFE_ZERO_INIT static StaticTimer_t timerBuffer[LED_COUNT];

static xSemaphoreHandle ledseqMutex;
static xQueueHandle ledseqCmdQueue;

static bool didInit;
static bool ledseqEnabled;

static void lesdeqCmdTask(void* param) 
{
    struct ledseqCmd_s command = {};

    while (true) {

        xQueueReceive(ledseqCmdQueue, &command, portMAX_DELAY);

        switch(command.command) {

            case run:
                ledseqRunBlocking(command.sequence);
                break;

            case stop:
                ledseqStopBlocking(command.sequence);
                break;
        }
    }
}


static void updateActive(const uint8_t ledPin) 
{
    activeSeq[ledPin] = NULL;

    ledSet(ledPin, false);

    for (ledseqContext_t* sequence = sequences; sequence != 0; 
            sequence = sequence->nextContext) {
        if (sequence->led == ledPin && sequence->state != LEDSEQ_STOP) {
            activeSeq[ledPin] = sequence;
            break;
        }
    }
}


static void ledseqEnable(const bool enable) 
{
    ledseqEnabled = enable;
}

static void ledseqRegisterSequence(ledseqContext_t* context) 
{
    context->state = LEDSEQ_STOP;
    context->nextContext = NULL;

    if (sequences == NULL) {
        sequences = context;
    } else {
        ledseqContext_t* last = sequences;
        if (last == context) {
            // Skip if already registered
            return;
        }

        while (last->nextContext != NULL) {
            last = last->nextContext;
            if (last == context) {
                // Skip if already registered
                return;
            }
        }

        last->nextContext = context;
    }
}

/* Center of the led sequence machine. This function is executed by the FreeRTOS
 * timers and runs the sequences
 */
static void runLedseq(xTimerHandle xTimer) 
{
    if (!ledseqEnabled) {
        return;
    }

    uint8_t led_id = 0;

    while (led_id < LED_COUNT) {
        if (xTimer == timer[led_id]) {
            break;
        }
        led_id++;
    }

    ledseqContext_t* context = activeSeq[led_id];

    if (NULL == context) {
        return;
    }

    auto leave = false;

    while(!leave) {

        if (context->state == LEDSEQ_STOP) {
            return;
        }

        const ledseqStep_t* step = &context->sequence[context->state];

        xSemaphoreTake(ledseqMutex, portMAX_DELAY);
        context->state++;
        auto ledPin = context->led;

        switch(step->action) {
            case LEDSEQ_LOOP:
                context->state = 0;
                break;
            case LEDSEQ_STOP:
                context->state = LEDSEQ_STOP;
                updateActive(ledPin);
                break;
            default:  //The step is a LED action and a time
                ledSet(ledPin, step->value);
                if (step->action == 0) {
                    break;
                }
                xTimerChangePeriod(xTimer, M2T(step->action), 0);
                xTimerStart(xTimer, 0);
                leave = true;
                break;
        }
        xSemaphoreGive(ledseqMutex);
    }
}

////////////////////////////////////////////////////////////////////////////

void ledseqInit() 
{
    if(didInit) {
        return;
    }

    ledInit();

    // Led sequence priority
    ledseqRegisterSequence(&seq_testPassed);
    ledseqRegisterSequence(&seq_testFailed);
    ledseqRegisterSequence(&seq_lowbat);
    ledseqRegisterSequence(&seq_charged);
    ledseqRegisterSequence(&seq_charging);
    ledseqRegisterSequence(&seq_calibrated);
    ledseqRegisterSequence(&seq_alive);
    ledseqRegisterSequence(&seq_linkUp);
    ledseqRegisterSequence(&seq_linkDown);

    //Initialise the sequences state
    for(int i=0; i<LED_COUNT; i++) {
        activeSeq[i] = 0;
    }

    //Init the soft timers that runs the led sequences for each leds
    for(int i=0; i<LED_COUNT; i++) {
        timer[i] = xTimerCreateStatic("ledseqTimer", M2T(1000), pdFALSE, (void*)i, 
                runLedseq, &timerBuffer[i]);
    }

    ledseqMutex = xSemaphoreCreateMutex();

    ledseqCmdQueue = xQueueCreate(10, sizeof(struct ledseqCmd_s));

    xTaskCreate(lesdeqCmdTask, LEDSEQCMD_TASK_NAME, LEDSEQCMD_TASK_STACKSIZE, 
            NULL, LEDSEQCMD_TASK_PRI, NULL);

    didInit = true;
}

bool ledseqTest(void) 
{
    bool status;

    status = didInit & ledTest();
    ledseqEnable(true);

    return status;
}

bool ledseqRun(ledseqContext_t *context) 
{
    struct ledseqCmd_s command;
    command.command = run;
    command.sequence = context;
    if (xQueueSend(ledseqCmdQueue, &command, 0) == pdPASS) {
        return true;
    }
    return false;
}

void ledseqRunBlocking(ledseqContext_t *context) 
{
    const uint8_t ledPin = context->led;

    xSemaphoreTake(ledseqMutex, portMAX_DELAY);
    context->state = 0;  //Reset the seq. to its first step
    updateActive(ledPin);
    xSemaphoreGive(ledseqMutex);

    // Run the first step if the new seq is the active sequence
    if(activeSeq[ledPin] == context) {
        runLedseq(timer[ledPin]);
    }
}

void ledseqSetChargeLevel(const float chargeLevel) 
{
    int onTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA * chargeLevel;
    int offTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA - onTime;

    seq_charging.sequence[0].action = onTime;
    seq_charging.sequence[1].action = offTime;
}

bool ledseqStop(ledseqContext_t *context) 
{
    struct ledseqCmd_s command;
    command.command = stop;
    command.sequence = context;
    if (xQueueSend(ledseqCmdQueue, &command, 0) == pdPASS) {
        return true;
    }
    return false;
}

void ledseqStopBlocking(ledseqContext_t *context) 
{

    const auto ledPin = context->led;

    xSemaphoreTake(ledseqMutex, portMAX_DELAY);
    context->state = LEDSEQ_STOP;  //Stop the seq.
    updateActive(ledPin);
    xSemaphoreGive(ledseqMutex);

    //Run the next active sequence (if any...)
    runLedseq(timer[ledPin]);
}

void ledseqShowFailure(void)
{
    ledseqRun(&seq_testFailed);
}

void ledseqShowSuccess(void)
{
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);
}

void ledseqShowCalibrated(void)
{
    ledseqRun(&seq_calibrated);
}

void ledseqShowCharged(void)
{
    ledseqStop(&seq_charging);
    ledseqRunBlocking(&seq_charged);
}

void ledseqShowCharging(void)
{
    ledseqStop(&seq_lowbat);
    ledseqStop(&seq_charged);
    ledseqRunBlocking(&seq_charging);
}

void ledseqShowLowPower(void)
{
    ledseqRunBlocking(&seq_lowbat);
}

void ledseqShowBattery(void)
{
    ledseqRunBlocking(&seq_charging);
    ledseqRun(&seq_charged);
}

void ledseqShowLinkUp(void)
{
    ledseqRun(&seq_linkUp);
}

void ledseqShowLinkDown(void)
{
    ledseqRun(&seq_linkDown);
}
