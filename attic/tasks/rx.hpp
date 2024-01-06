/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
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

#include <stdlib.h>

#include <free_rtos.h>
#include <task.h>


// Arduino library
#include <dsmrx.hpp>

#include <crossplatform.h>
#include <datatypes.h>

class RxTask {

    public:

        demands_t demands;

        bool isAuxSet;

        uint32_t timestamp;

        void init(void)
        {
            if (_didInit) {
                return;
            }

            xTaskCreateStatic(
                    dsmrxTask, 
                    "DSMRX", 
                    STACKSIZE, 
                    this, 
                    0, 
                    _stackBuffer,
                    &_taskBuffer);


            _didInit = true;
        }

     private:

        static const auto STACKSIZE = configMINIMAL_STACK_SIZE;

        static const uint8_t NCHAN = 8;

        StackType_t  _stackBuffer[STACKSIZE]; 

        StaticTask_t _taskBuffer;

        Dsm2048 _dsmrx;

        bool _didInit;

        float _chanvals[NCHAN];

        uint32_t _timestamp;

        static void dsmrxTask(void *param)
        {
            ((RxTask *)param)->run();
        }

        void run(void)
        {
            systemWaitStart();


            _dsmrx.init();

            while (true) {

                uint8_t byte = 0;

                auto gotByte = serial1Read(&byte);

                if (gotByte) {

                    auto time = (uint32_t)micros();

                    _dsmrx.parse(byte, time);

                    if (_dsmrx.gotNewFrame()) {

                        timestamp = time;

                        _dsmrx.getChannelValues(_chanvals, NCHAN);

                        demands.thrust =  _chanvals[0];
                        demands.roll =   -_chanvals[1]; // note negation
                        demands.pitch =   _chanvals[2];
                        demands.yaw =     _chanvals[3];

                        isAuxSet = _chanvals[4] > 0.5f;

                        timestamp = _timestamp;

                    }
                }

            }        
        }


};
