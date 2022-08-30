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

#include <string.h>

#include "dshot_bitbang.h"
#include "io_types.h"
#include "timer.h"
#include "esc.h"

class DshotEsc : public Esc {

    private:

        static const uint16_t MIN_VALUE = 48;
        static const uint16_t MAX_VALUE = 2047;
        static const uint16_t STOP_VALUE = 0;
        static const uint16_t VALUE_RANGE = MAX_VALUE - MIN_VALUE;

        static const uint8_t ALL_MOTORS = 255;

        // Time to separate dshot beacon and armining/disarming events
        static const uint32_t BEACON_GUARD_DELAY_US = 1200000;  


        typedef struct {

            float    (*convertFromExternal)(uint16_t value);
            uint16_t (*convertToExternal)(float value);
            void     (*disable)(void);
            bool     (*enable)(void);
            bool     (*isEnabled)(uint8_t index);
            void     (*postInit)(void);
            void     (*shutdown)(void);
            void     (*updateComplete)(void);
            bool     (*updateStart)(void);
            void     (*write)(uint8_t index, float value);
            void     (*writeInt)(uint8_t index, uint16_t value);

        } escVTable_t;

        typedef struct {
            volatile timCCR_t *ccr;
            TIM_TypeDef       *tim;
        } timerChannel_t;

        typedef struct {
            //timerChannel_t channel;
            float pulseScale;
            float pulseOffset;
            bool forceOverflow;
            bool enabled;
            //IO_t io;
        } pwmOutputPort_t;

        typedef enum {

            // dshot commands sent inline with motor signal (motors must be enabled)
            CMD_TYPE_INLINE,    

            // dshot commands sent in blocking method (motors must be disabled)
            CMD_TYPE_BLOCKING       

        } commandType_e;

        typedef enum {
            CMD_MOTOR_STOP,
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

        uint8_t     m_count;
        bool        m_enabled;
        uint32_t    m_enableTimeMs;
        bool        m_initialized;
        pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
        escVTable_t m_vTable;

        static void commandWrite(
                uint8_t index,
                commands_e command,
                commandType_e commandType)
        {
            (void)index;
            (void)command;
            (void)commandType;

            /* XXX
               if (!commandsAreEnabled(escDevice, commandType) ||
               (command > DSHOT_MAX_COMMAND) ||
               dshotCommandQueueFull()) { return;
               }

               uint8_t repeats = 1;
               uint32_t delayAfterCommandUs = DSHOT_COMMAND_DELAY_US;

               switch (command) {
               case CMD_SPIN_DIRECTION_1:
               case CMD_SPIN_DIRECTION_2:
               case CMD_SAVE_SETTINGS:
               case CMD_SPIN_DIRECTION_NORMAL:
               case CMD_SPIN_DIRECTION_REVERSED:
               repeats = 10;
               break;
               case CMD_BEACON1:
               case CMD_BEACON2:
               case CMD_BEACON3:
               case CMD_BEACON4:
               case CMD_BEACON5:
               delayAfterCommandUs = DSHOT_BEEP_DELAY_US;
               break;
               default:
               break;
               }

               if (commandType == CMD_TYPE_BLOCKING) {
               delayMicroseconds(DSHOT_INITIAL_DELAY_US - DSHOT_COMMAND_DELAY_US);
               for (; repeats; repeats--) {
               delayMicroseconds(DSHOT_COMMAND_DELAY_US);

               uint32_t timeoutUs = micros() + 1000;
               while (!escGetVTable(escDevice).updateStart() &&
               cmpTimeUs(timeoutUs, micros()) > 0);
               for (uint8_t i = 0; i < motorCount; i++) {
               if ((i == index) || (index == ALL_MOTORS)) {
               motorDmaOutput_t *const motor = getMotorDmaOutput(i);
               motor->protocolControl.requestTelemetry = true;
               escGetVTable(escDevice).writeInt(i, command);
               }
               }

               escGetVTable(escDevice).updateComplete();
               }
               delayMicroseconds(delayAfterCommandUs);
               } else if (commandType == CMD_TYPE_INLINE) {
               dshotCommandControl_t *commandControl = addCommand();
               if (commandControl) {
               commandControl->repeats = repeats;
               commandControl->delayAfterCommandUs = delayAfterCommandUs;
               for (unsigned i = 0; i < motorCount; i++) {
               if (index == i || index == ALL_MOTORS) {
               commandControl->command[i] = command;
               } else {
               commandControl->command[i] = CMD_MOTOR_STOP;
               }
               }
               if (allMotorsAreIdle(motorCount)) {
            // we can skip the motors idle wait state
            commandControl->state = DSHOT_COMMAND_STATE_STARTDELAY;
            commandControl->nextCommandCycleDelay =
            dshotCommandCyclesFromTime(DSHOT_INITIAL_DELAY_US);
            } else {
            commandControl->state = DSHOT_COMMAND_STATE_IDLEWAIT;

            // will be set after idle wait completes
            commandControl->nextCommandCycleDelay = 0;  
            }
            }
        }*/

        } // commandWrite


    public:

        DshotEsc(uint8_t count)
        {
            m_count = count;
        }

        virtual void begin(void) override 
        {
            memset(motors, 0, sizeof(motors));

            /* XXX
            escDevice = dshotBitbangDevInit(motorCount);

            escDevice->count = motorCount;
            escDevice->initialized = true;
            escDevice->enableTimeMs = 0;
            escDevice->enabled = false;
            */
        }

        virtual float  convertFromExternal(uint16_t value) override 
        {
            return m_vTable.convertFromExternal(value);
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
            commandWrite(
                    ALL_MOTORS,
                    CMD_SPIN_DIRECTION_NORMAL,
                    CMD_TYPE_INLINE);
        }

        virtual void write(float *values) override 
        {
            if (m_enabled) {
                if (!m_vTable.updateStart()) {
                    return;
                }
                for (auto i = 0; i < m_count; i++) {
                    m_vTable.write(i, values[i]);
                }
                m_vTable.updateComplete();
            }
        }            
};
