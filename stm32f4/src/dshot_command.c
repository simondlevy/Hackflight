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

#include <stdbool.h>
#include <stdint.h>


#include <esc.h>
#include <time.h>

#include "platform.h"
#include "io.h"
#include "escdev.h"
#include "timer.h"
#include "dshot.h"
#include "dshot_dpwm.h"
#include "pwm_output.h"
#include "dshot_command.h"
#include "systemdev.h"

#define DSHOT_PROTOCOL_DETECTION_DELAY_MS 3000
#define DSHOT_INITIAL_DELAY_US 10000
#define DSHOT_COMMAND_DELAY_US 1000
#define DSHOT_ESCINFO_DELAY_US 12000
#define DSHOT_BEEP_DELAY_US 100000
#define DSHOT_MAX_COMMANDS 3

#define FALLTHROUGH __attribute__ ((fallthrough))

typedef enum {
    DSHOT_COMMAND_STATE_IDLEWAIT,   // waiting for motors to go idle
    DSHOT_COMMAND_STATE_STARTDELAY, // initial delay before a sequence of commands
    DSHOT_COMMAND_STATE_ACTIVE,     // actively sending command
    DSHOT_COMMAND_STATE_POSTDELAY   // delay period after the command has been sent
} dshotCommandState_e;

typedef struct dshotCommandControl_s {
    dshotCommandState_e state;
    uint32_t nextCommandCycleDelay;
    uint32_t delayAfterCommandUs;
    uint8_t repeats;
    uint8_t command[MAX_SUPPORTED_MOTORS];
} dshotCommandControl_t;

// default to 8KHz (125us) loop to prevent possible div/0
static uint32_t dshotCommandPidLoopTimeUs = 125; 

// gets set to the actual value when the PID loop is initialized
static dshotCommandControl_t commandQueue[DSHOT_MAX_COMMANDS + 1];
static uint8_t commandQueueHead;
static uint8_t commandQueueTail;

void dshotSetPidLoopTime(uint32_t pidLoopTime)
{
    dshotCommandPidLoopTimeUs = pidLoopTime;
}

static  bool dshotCommandQueueFull()
{
    return (commandQueueHead + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueTail;
}

bool dshotCommandQueueEmpty(void)
{
    return commandQueueHead == commandQueueTail;
}

static  bool isLastDshotCommand(void)
{
    return ((commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueHead);
}

bool dshotCommandIsProcessing(void)
{
    if (dshotCommandQueueEmpty()) {
        return false;
    }
    dshotCommandControl_t* command = &commandQueue[commandQueueTail];
    const bool commandIsProcessing = command->state ==
        DSHOT_COMMAND_STATE_STARTDELAY || command->state ==
        DSHOT_COMMAND_STATE_ACTIVE || (command->state ==
                DSHOT_COMMAND_STATE_POSTDELAY && !isLastDshotCommand()); return
        commandIsProcessing;
}

static  bool dshotCommandQueueUpdate(void)
{
    if (!dshotCommandQueueEmpty()) {
        commandQueueTail = (commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1);
        if (!dshotCommandQueueEmpty()) {
            // There is another command in the queue so update it so it's ready
            // to output in sequence. It can go directly to the
            // DSHOT_COMMAND_STATE_ACTIVE state and bypass the
            // DSHOT_COMMAND_STATE_IDLEWAIT and DSHOT_COMMAND_STATE_STARTDELAY
            // states.
            dshotCommandControl_t* nextCommand = &commandQueue[commandQueueTail];
            nextCommand->state = DSHOT_COMMAND_STATE_ACTIVE;
            nextCommand->nextCommandCycleDelay = 0;
            return true;
        }
    }
    return false;
}

static  uint32_t dshotCommandCyclesFromTime(uint32_t delayUs)
{
    // Find the minimum number of motor output cycles needed to
    // provide at least delayUs time delay

    return (delayUs + dshotCommandPidLoopTimeUs - 1) / dshotCommandPidLoopTimeUs;
}

static dshotCommandControl_t* addCommand()
{
    int newHead = (commandQueueHead + 1) % (DSHOT_MAX_COMMANDS + 1);
    if (newHead == commandQueueTail) {
        return NULL;
    }
    dshotCommandControl_t* control = &commandQueue[commandQueueHead];
    commandQueueHead = newHead;
    return control;
}

static bool allMotorsAreIdle(uint8_t motorCount)
{
    for (unsigned i = 0; i < motorCount; i++) {
        const motorDmaOutput_t *motor = getMotorDmaOutput(i);
        if (motor->protocolControl.value) {
            return false;
        }
    }

    return true;
}

static bool commandsAreEnabled(void * escDevice)
{
    return escIsEnabled(escDevice) &&
        escGetEnableTimeMs(escDevice) &&
        millis() > escGetEnableTimeMs(escDevice) +
        DSHOT_PROTOCOL_DETECTION_DELAY_MS;
}

uint8_t dshotCommandGetCurrent(uint8_t index)
{
    return commandQueue[commandQueueTail].command[index];
}

// This function is used to synchronize the dshot command output timing with
// the normal motor output timing tied to the PID loop frequency. A "true"
// result allows the motor output to be sent, "false" means delay until next
// loop. So take the example of a dshot command that needs to repeat 10 times
// at 1ms intervals.  If we have a 8KHz PID loop we'll end up sending the dshot
// command every 8th motor output.
bool dshotCommandOutputIsEnabled(uint8_t motorCount)
{
    UNUSED(motorCount);

    dshotCommandControl_t* command = &commandQueue[commandQueueTail];
    switch (command->state) {
        case DSHOT_COMMAND_STATE_IDLEWAIT:
            if (allMotorsAreIdle(motorCount)) {
                command->state = DSHOT_COMMAND_STATE_STARTDELAY;
                command->nextCommandCycleDelay =
                    dshotCommandCyclesFromTime(DSHOT_INITIAL_DELAY_US);
            }
            break;

        case DSHOT_COMMAND_STATE_STARTDELAY:
            if (command->nextCommandCycleDelay) {
                --command->nextCommandCycleDelay;
                return false;  // Delay motor output until start of command sequence
            }
            command->state = DSHOT_COMMAND_STATE_ACTIVE;
            command->nextCommandCycleDelay = 0;  // first iter of repeat happens now
            FALLTHROUGH;

        case DSHOT_COMMAND_STATE_ACTIVE:
            if (command->nextCommandCycleDelay) {
                --command->nextCommandCycleDelay;
                return false;  // Delay motor output until the next command repeat
            }

            command->repeats--;
            if (command->repeats) {
                command->nextCommandCycleDelay =
                    dshotCommandCyclesFromTime(DSHOT_COMMAND_DELAY_US);
            } else {
                command->state = DSHOT_COMMAND_STATE_POSTDELAY;
                command->nextCommandCycleDelay =
                    dshotCommandCyclesFromTime(command->delayAfterCommandUs);
                if (!isLastDshotCommand() && command->nextCommandCycleDelay > 0) {
                    // Account for the 1 extra motor output loop between
                    // commands.  Otherwise the inter-command delay will be
                    // DSHOT_COMMAND_DELAY_US + 1 loop.
                    command->nextCommandCycleDelay--;
                }
            }
            break;

        case DSHOT_COMMAND_STATE_POSTDELAY:
            if (command->nextCommandCycleDelay) {
                --command->nextCommandCycleDelay;
                return false;  // Delay motor output until end of post-command delay
            }
            if (dshotCommandQueueUpdate()) {
                // Will be true if the command queue is not empty and we
                // want to wait for the next command to start in sequence.
                return false;
            }
    }

    return true;
}
