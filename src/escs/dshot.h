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
#include "escdev.h"

class DshotEsc : public Esc {

    private:

        static const uint16_t MIN_VALUE = 48;
        static const uint16_t MAX_VALUE = 2047;
        static const uint16_t STOP_VALUE = 0;
        static const uint16_t VALUE_RANGE = MAX_VALUE - MIN_VALUE;

        static const uint8_t ALL_MOTORS = 255;

        // Time to separate dshot beacon and armining/disarming events
        static const uint32_t BEACON_GUARD_DELAY_US = 1200000;  

        static const uint32_t COMMAND_DELAY_US = 1000;

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

        typedef enum {
            COMMAND_STATE_IDLEWAIT,   // waiting for motors to go idle
            COMMAND_STATE_STARTDELAY, // initial delay before a sequence of commands
            COMMAND_STATE_ACTIVE,     // actively sending command
            COMMAND_STATE_POSTDELAY   // delay period after the command has been sent
        } commandState_e;

        typedef struct {
            commandState_e state;
            uint32_t nextCommandCycleDelay;
            uint32_t delayAfterCommandUs;
            uint8_t repeats;
            uint8_t command[MAX_SUPPORTED_MOTORS];
        } commandControl_t;

        uint8_t     m_motorCount;

        escDevice_t * m_escDevice;

        pwmOutputPort_t m_motors[MAX_SUPPORTED_MOTORS];

    public:

        DshotEsc(uint8_t count) 
            : Esc(count)
        {
        }

        virtual void begin(void) override 
        {
            memset(m_motors, 0, sizeof(m_motors));

            m_escDevice = dshotBitbangDevInit(m_motorCount);

            m_escDevice->count = m_motorCount;
            m_escDevice->initialized = true;
            m_escDevice->enableTimeMs = 0;
            m_escDevice->enabled = false;
        }

        virtual float  convertFromExternal(uint16_t value) override 
        {
            return m_escDevice->vTable.convertFromExternal(value);
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
            /*
            uint8_t repeats = 10;
            uint32_t delayAfterCommandUs = COMMAND_DELAY_US;

            dshotCommandControl_t *commandControl = addCommand();

            if (commandControl) {
                commandControl->repeats = repeats;
                commandControl->delayAfterCommandUs = delayAfterCommandUs;
                for (unsigned i = 0; i < m_motorCount; i++) {
                    commandControl->command[i] = DSHOT_CMD_SPIN_DIRECTION_NORMAL;
                }
                if (allMotorsAreIdle(m_motorCount)) {
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
            */
        }

        virtual void write(float *values) override 
        {
            if (m_escDevice->enabled) {
                if (!m_escDevice->vTable.updateStart()) {
                    return;
                }
                for (auto i = 0; i < m_motorCount; i++) {
                    m_escDevice->vTable.write(i, values[i]);
                }
                m_escDevice->vTable.updateComplete();
            }
        }            
};
