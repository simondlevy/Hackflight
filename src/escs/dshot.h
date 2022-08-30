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

#include "esc.h"

class DshotEsc : public Esc {

    private:

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

        uint8_t     m_count;
        bool        m_enabled;
        uint32_t    m_enableTimeMs;
        bool        m_initialized;
        escVTable_t m_vTable;

    public:

        DshotEsc(uint8_t count) override 
        {
            // XXX
            (void)count;
        }

        virtual void begin(void) override 
        {
            // XXX
        }

        virtual float  convertFromExternal(uint16_t value) override 
        {
            // XXX
            (void)value;
            return 0;
        }

        virtual bool isProtocolDshot(void) override 
        {
            return true;
        }

        virtual bool isReady(uint32_t currentTime) override 
        {
            // XXX
            (void)currentTime;
            return false;
        }

        virtual float valueDisarmed(void) override 
        {
            // XXX
            return 0;
        }

        virtual float valueHigh(void) override 
        {
            // XXX
            return 0;
        }

        virtual float valueLow(void) override 
        {
            // XXX
            return 0;
        }

        virtual void stop(void) override 
        {
            // XXX
        }

        virtual void write(float *values) override 
        {
            // XXX
            (void)values;
        }
};

#endif
