/**
 * Copyright (C) 2011-2018 Bitcraze AB, 2025 Simon D. Levy
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

#pragma once

// #include <TinyPICO.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

#include <SPI.h>

#define DOTSTAR_PWR 13
#define DOTSTAR_DATA 2
#define DOTSTAR_CLK 12

#define BAT_CHARGE 34
#define BAT_VOLTAGE 35

class FreeRtosTask {

    private:

        static const auto STACKSIZE = 3 * configMINIMAL_STACK_SIZE; // arbitrary

        typedef void (*taskfun_t)(void * obj);

        StackType_t _taskStackBuffer[STACKSIZE]; 

        StaticTask_t _taskTaskBuffer;

        bool _didInit;

    public:

        void init(
                const taskfun_t fun,
                const char * name,
                void * obj,
                const uint8_t priority
                )
        {
            xTaskCreateStatic(
                    fun, 
                    name, 
                    STACKSIZE, 
                    obj, 
                    priority, 
                    _taskStackBuffer,
                    &_taskTaskBuffer);

            _didInit = true;
        }

        bool didInit()
        {
            return _didInit;
        }
};
