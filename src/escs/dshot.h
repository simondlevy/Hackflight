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

#include "core/clock.h"
#include "esc.h"

#include <timer.h>
#include <io_types.h>

extern "C" {
    void * dshotInit(uint8_t count, uint32_t period);
    void   dshotStop(void * escDevice);
}

class DshotEsc : public Esc {

    private:

        static const uint16_t MIN_VALUE = 48;
        static const uint16_t MAX_VALUE = 2047;
        static const uint16_t STOP_VALUE = 0;
        static const uint16_t VALUE_RANGE = MAX_VALUE - MIN_VALUE;

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
            uint8_t     count;
            bool        enabled;
            uint32_t    enableTimeMs;
            bool        initialized;
            escVTable_t vTable;
        } escDevice_t;

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

        void * m_escDevice;

    pwmOutputPort_t m_motors[MAX_SUPPORTED_MOTORS];

    public:

        DshotEsc(uint8_t count) 
            : Esc(count)
        {
        }

        virtual void begin(void) override 
        {
            m_escDevice = dshotInit(m_motorCount, Clock::PERIOD());
        }

        virtual float convertFromExternal(uint16_t value) override 
        {
            escDevice_t * escDevice = (escDevice_t *)m_escDevice;
            return escDevice->vTable.convertFromExternal(value);
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
            dshotStop(m_escDevice);
        }

        virtual void write(float *values) override
        {
            escDevice_t * escDevice = (escDevice_t *)m_escDevice;

            if (escDevice->enabled) {
                if (!escDevice->vTable.updateStart()) {
                    return;
                }
                for (auto i=0; i <escDevice->count; i++) {
                    escDevice->vTable.write(i, values[i]);
                }
                escDevice->vTable.updateComplete();
            }
        }

}; // class DshotEsc
