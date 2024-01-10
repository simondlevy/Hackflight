/*
   Copyright (c) 2024 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include <datatypes.h>
#include <hackflight.hpp>
#include <msp.hpp>
#include <tasks/free_rtos.hpp>
#include <tasks/free_rtos/core.hpp>

class VisualizerTask : public FreeRTOSTask {

    public:

        void init(CoreTask * coreTask)
        {
            if (didInit) {
                return;
            }

            _coreTask= coreTask;

            FreeRTOSTask::init(runVisualizerTask, "VISUALIZER", this, 1);

            didInit = true;
        }

        bool parse(
                //ReceiverTask & receiverTask,
                const uint8_t byte)
        {
            switch (_msp.parse(byte)) {

                case 105: // RC
                    {
                        int16_t channels[6] = { /*
                            (int16_t)receiverTask.getRawThrottle(),
                            (int16_t)receiverTask.getRawRoll(),
                            (int16_t)receiverTask.getRawPitch(),
                            (int16_t)receiverTask.getRawYaw(),
                            (int16_t)receiverTask.getRawAux1(),
                            (int16_t)receiverTask.getRawAux2()*/
                        };

                        serializeShorts(105, channels, 6);

                    } 
                    return true;

                case 108: // ESTIMATOR
                    {
                        const auto vehicleState = _coreTask->vehicleState;
                        const int16_t angles[3] = {
                            (int16_t)(10 * vehicleState.phi),
                            (int16_t)(10 * vehicleState.theta),
                            (int16_t)vehicleState.psi
                        };

                        serializeShorts(108, angles, 3);
                    } 
                    return true;

                case 214: // SET_MOTORS
                    {
                        readAndConvertMotor(0);
                        readAndConvertMotor(1);
                        readAndConvertMotor(2);
                        readAndConvertMotor(3);
                    } 
                    break;

                default:
                    break;
            }

            return false;
        }

        float motors[Hackflight::MAX_MOTOR_COUNT];


    private:

        static float scale(const float value)
        {
            return 1000 + 1000 * value;
        }

        static void runVisualizerTask(void* obj)
        {
            ((VisualizerTask *)obj)->run();
        }

        Msp _msp;

        CoreTask * _coreTask;

        void run(void)
        {
            while (true) {

                while (Serial.available()) {

                }
            }
        }

        void serializeShorts(
                const uint8_t messageType,
                const int16_t src[],
                const uint8_t count)
        {
            _msp.serializeShorts(messageType, src, count);
        }

        void  readAndConvertMotor(const uint8_t index)
        {
            motors[index] = (_msp.parseShort(index) - 1000) / 1000.;
        }

};
