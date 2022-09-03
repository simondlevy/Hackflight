/*
   Copyright (c) 2022 Simon D. Levy

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

#include <stdbool.h>
#include <stdint.h>

#include <core/clock.h>
#include <core/constrain.h>
#include <esc.h>
#include <escs/dshot_dev.h>
#include <escs/dshot_protocol.h>
#include <maxmotors.h>
#include <pwm.h>
#include <time.h>
#include <timer.h>
#include <io_types.h>
#include <time.h>

#include "platform.h"
#include "io.h"
#include "timer.h"
#include "systemdev.h"

#define DSHOT_PROTOCOL_DETECTION_DELAY_MS 3000
#define DSHOT_INITIAL_DELAY_US 10000
#define DSHOT_COMMAND_DELAY_US 1000
#define DSHOT_ESCINFO_DELAY_US 12000
#define DSHOT_BEEP_DELAY_US 100000
#define DSHOT_MAX_COMMANDS 3

#define FALLTHROUGH __attribute__ ((fallthrough))

#define DSHOT_MAX_COMMAND 47

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

motorDmaOutput_t m_dmaMotors[MAX_SUPPORTED_MOTORS];

static  bool isLastDshotCommand(void)
{
    return ((commandQueueTail + 1) % (DSHOT_MAX_COMMANDS + 1) == commandQueueHead);
}

static bool dshotCommandQueueEmpty(void)
{
    return commandQueueHead == commandQueueTail;
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


static motorDmaOutput_t * getMotorDmaOutput(uint8_t index)
{
    return &m_dmaMotors[index];
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

static bool dshotCommandIsProcessing(void)
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

static uint8_t dshotCommandGetCurrent(uint8_t index)
{
    return commandQueue[commandQueueTail].command[index];
}

// This function is used to synchronize the dshot command output timing with
// the normal motor output timing tied to the PID loop frequency. A "true"
// result allows the motor output to be sent, "false" means delay until next
// loop. So take the example of a dshot command that needs to repeat 10 times
// at 1ms intervals.  If we have a 8KHz PID loop we'll end up sending the dshot
// command every 8th motor output.
static bool dshotCommandOutputIsEnabled(uint8_t motorCount)
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

// -----------------------------------------------------------------------------

class DshotEsc : public Esc {

    private:

        static const uint16_t MIN_VALUE = 48;
        static const uint16_t MAX_VALUE = 2047;
        static const uint16_t STOP_VALUE = 0;
        static const uint16_t VALUE_RANGE = MAX_VALUE - MIN_VALUE;

        // Time to separate dshot beacon and armining/disarming events
        static const uint32_t BEACON_GUARD_DELAY_US = 1200000;  

        static const uint32_t COMMAND_DELAY_US = 1000;

        static const uint32_t INITIAL_DELAY_US = 10000;

        static const uint8_t MAX_COMMANDS = 3;

        // default to 8KHz (125us) loop to prevent possible div/0
        static const uint32_t PID_LOOP_TIME_US = 125; 

        typedef enum {
            CMD_MOTOR_STOP = 0,
            CMD_BEACON1,
            CMD_BEACON2,
            CMD_BEACON3,
            CMD_BEACON4,
            CMD_BEACON5,
            CMD_ESC_INFO, // V2 includes settings
            CMD_SPIN_DIRECTION_1,
            CMD_SPIN_DIRECTION_2,
            CMD_SETTINGS_REQUEST, // Currently not implemented
            CMD_SAVE_SETTINGS,
            CMD_SPIN_DIRECTION_NORMAL = 20,
            CMD_SPIN_DIRECTION_REVERSED = 21,
            CMD_LED0_ON, // BLHeli32 only
            CMD_LED1_ON, // BLHeli32 only
            CMD_LED2_ON, // BLHeli32 only
            CMD_LED3_ON, // BLHeli32 only
            CMD_LED0_OFF, // BLHeli32 only
            CMD_LED1_OFF, // BLHeli32 only
            CMD_LED2_OFF, // BLHeli32 only
            CMD_LED3_OFF, // BLHeli32 only
            CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
            CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
            CMD_MAX = 47
        } commands_e;

        typedef struct {
            volatile timCCR_t * ccr;
            TIM_TypeDef * tim;
        } timerChannel_t;

        typedef struct {
            timerChannel_t channel;
            float pulseScale;
            float pulseOffset;
            bool forceOverflow;
            bool enabled;
            IO_t io;
        } pwmOutputPort_t;

        typedef enum {
            COMMAND_STATE_IDLEWAIT,   // waiting for motors to go idle
            COMMAND_STATE_STARTDELAY, // initial delay before a sequence of cmds
            COMMAND_STATE_ACTIVE,     // actively sending command
            COMMAND_STATE_POSTDELAY   // delay period after the cmd has been sent
        } commandState_e;

        typedef struct dshotCommandControl_s {
            commandState_e state;
            uint32_t nextCommandCycleDelay;
            uint32_t delayAfterCommandUs;
            uint8_t repeats;
            uint8_t command[MAX_SUPPORTED_MOTORS];
        } commandControl_t;

        // gets set to the actual value when the PID loop is initialized
        commandControl_t m_commandQueue[MAX_COMMANDS + 1];
        uint8_t m_commandQueueHead;
        uint8_t m_commandQueueTail;

        bool m_enabled;

        commandControl_t * addCommand(void)
        {
            auto newHead = (m_commandQueueHead + 1) % (MAX_COMMANDS + 1);
            if (newHead == m_commandQueueTail) {
                return NULL;
            }
            auto * control = &m_commandQueue[m_commandQueueHead];
            m_commandQueueHead = newHead;
            return control;
        }

        bool allMotorsAreIdle(void)
        {
            for (auto i=0; i<m_motorCount; i++) {
                const motorDmaOutput_t *motor = getMotorDmaOutput(i);
                if (motor->protocolControl.value) {
                    return false;
                }
            }

            return true;
        }

        static uint32_t commandCyclesFromTime(uint32_t delayUs)
        {
            // Find the minimum number of motor output cycles needed to
            // provide at least delayUs time delay

            return (delayUs + PID_LOOP_TIME_US - 1) / PID_LOOP_TIME_US;
        }

        static uint32_t dshotCommandCyclesFromTime(uint32_t delayUs)
        {
            // Find the minimum number of motor output cycles needed to
            // provide at least delayUs time delay

            return (delayUs + Clock::PERIOD() - 1) / Clock::PERIOD();
        }

        static float scaleRangef(
                float x, float srcFrom, float srcTo, float destFrom, float destTo)
        {
            auto a = (destTo - destFrom) * (x - srcFrom);
            auto b = srcTo - srcFrom;
            return (a / b) + destFrom;
        }


    protected:

        dshotProtocol_t m_protocol;

        uint8_t m_motorPins[MAX_SUPPORTED_MOTORS];
        uint8_t m_motorCount;

        virtual void deviceInit(void) = 0;
        virtual bool enable(void) = 0;
        virtual void postInit(void) = 0;
        virtual void updateComplete(void) = 0;
        virtual bool updateStart(void) = 0;
        virtual void write(uint8_t index, float value) = 0;

        DshotEsc(vector<uint8_t> * pins, dshotProtocol_t protocol=DSHOT600) 
            : Esc(pins)
        {
            m_protocol = protocol;

            m_motorCount = pins->size();

            for (auto i=0; i<m_motorCount; ++i) {
                m_motorPins[i] = (*pins)[i];
            }
        }

        bool commandQueueEmpty(void)
        {
            return m_commandQueueHead == m_commandQueueTail;
        }

        bool commandQueueUpdate(void)
        {
            if (!commandQueueEmpty()) {
                m_commandQueueTail = (m_commandQueueTail + 1) % (MAX_COMMANDS + 1);
                if (!commandQueueEmpty()) {
                    // There is another command in the queue so update it so it's ready
                    // to output in sequence. It can go directly to the
                    // DSHOT_COMMAND_STATE_ACTIVE state and bypass the
                    // DSHOT_COMMAND_STATE_IDLEWAIT and DSHOT_COMMAND_STATE_STARTDELAY
                    // states.
                    commandControl_t* nextCommand = &m_commandQueue[m_commandQueueTail];
                    nextCommand->state = COMMAND_STATE_ACTIVE;
                    nextCommand->nextCommandCycleDelay = 0;
                    return true;
                }
            }
            return false;
        }

        // This function is used to synchronize the dshot command output timing with
        // the normal motor output timing tied to the PID loop frequency. A "true"
        // result allows the motor output to be sent, "false" means delay until next
        // loop. So take the example of a dshot command that needs to repeat 10 times
        // at 1ms intervals.  If we have a 8KHz PID loop we'll end up sending the dshot
        // command every 8th motor output.
        bool commandOutputIsEnabled(void)
        {
            commandControl_t* command = &m_commandQueue[m_commandQueueTail];

            switch (command->state) {
                case COMMAND_STATE_IDLEWAIT:
                    if (allMotorsAreIdle()) {
                        command->state = COMMAND_STATE_STARTDELAY;
                        command->nextCommandCycleDelay =
                            dshotCommandCyclesFromTime(INITIAL_DELAY_US);
                    }
                    break;

                case COMMAND_STATE_STARTDELAY:
                    if (command->nextCommandCycleDelay) {
                        --command->nextCommandCycleDelay;
                        return false;  // Delay motor output till start of command sequence
                    }
                    command->state = COMMAND_STATE_ACTIVE;
                    command->nextCommandCycleDelay = 0;  // first iter of repeat happens now
                    [[fallthrough]];

                case COMMAND_STATE_ACTIVE:
                    if (command->nextCommandCycleDelay) {
                        --command->nextCommandCycleDelay;
                        return false;  // Delay motor output until the next command repeat
                    }

                    command->repeats--;
                    if (command->repeats) {
                        command->nextCommandCycleDelay =
                            commandCyclesFromTime(COMMAND_DELAY_US);
                    } else {
                        command->state = COMMAND_STATE_POSTDELAY;
                        command->nextCommandCycleDelay =
                            commandCyclesFromTime(command->delayAfterCommandUs);
                        if (!isLastCommand() && command->nextCommandCycleDelay > 0) {
                            // Account for the 1 extra motor output loop between
                            // commands.  Otherwise the inter-command delay will be
                            // COMMAND_DELAY_US + 1 loop.
                            command->nextCommandCycleDelay--;
                        }
                    }
                    break;

                case COMMAND_STATE_POSTDELAY:
                    if (command->nextCommandCycleDelay) {
                        --command->nextCommandCycleDelay;
                        return false;  // Delay motor output until end of post-command delay
                    }
                    if (commandQueueUpdate()) {
                        // Will be true if the command queue is not empty and we
                        // want to wait for the next command to start in sequence.
                        return false;
                    }
                    break;
            }

            return true;
        }

        bool isLastCommand(void)
        {
            return ((m_commandQueueTail + 1) % (MAX_COMMANDS + 1) == m_commandQueueHead);
        }

        uint8_t commandGetCurrent(uint8_t index)
        {
            return m_commandQueue[m_commandQueueTail].command[index];
        }

        bool commandIsProcessing(void)
        {
            if (commandQueueEmpty()) {
                return false;
            }

            commandControl_t* command = &m_commandQueue[m_commandQueueTail];

            return
                command->state == COMMAND_STATE_STARTDELAY ||
                command->state == COMMAND_STATE_ACTIVE ||
                (command->state == COMMAND_STATE_POSTDELAY && !isLastCommand()); 
        }

    public:

        virtual void begin(void) override 
        {
            deviceInit();

            m_enabled = false;

            postInit();

            if (enable()) {
                m_enabled = true;
            }
        }

        virtual float convertFromExternal(uint16_t value) override 
        {
            auto constrainedValue = constrain_u16(value, PWM_MIN, PWM_MAX);

            return constrainedValue == PWM_MIN ?
                (float)CMD_MOTOR_STOP :
                scaleRangef(constrainedValue, PWM_MIN + 1, PWM_MAX,
                        MIN_VALUE, MAX_VALUE);
        }


        virtual bool isProtocolDshot(void) override 
        {
            return true;
        }

        virtual bool isReady(uint32_t currentTimeUs) override 
        {
            return currentTimeUs >= BEACON_GUARD_DELAY_US;
        }

        virtual float valueDisarmed(void) override 
        {
            return (float)STOP_VALUE;
        }

        virtual float valueHigh(void) override 
        {
            return MAX_VALUE;
        }

        virtual float valueLow(void) override 
        {
            return MIN_VALUE + 0.045 * VALUE_RANGE;
        }

        virtual void stop(void) override 
        {
            commandControl_t *commandControl = addCommand();

            if (commandControl) {
                commandControl->repeats = 10;
                commandControl->delayAfterCommandUs = COMMAND_DELAY_US;
                for (auto i=0; i<m_motorCount; i++) {
                    commandControl->command[i] = CMD_SPIN_DIRECTION_NORMAL;
                }
                if (allMotorsAreIdle()) {
                    // we can skip the motors idle wait state
                    commandControl->state = COMMAND_STATE_STARTDELAY;
                    commandControl->nextCommandCycleDelay =
                        dshotCommandCyclesFromTime(INITIAL_DELAY_US);
                } else {
                    commandControl->state = COMMAND_STATE_IDLEWAIT;

                    // will be set after idle wait completes
                    commandControl->nextCommandCycleDelay = 0;  
                }
            }
        }

        virtual void write(float *values) override
            {
                if (m_enabled) {
                    if (!updateStart()) {
                        return;
                    }
                    for (auto i=0; i <m_motorCount; i++) {
                        write(i, values[i]);
                    }
                    updateComplete();
            }
        }

}; // class DshotEsc
