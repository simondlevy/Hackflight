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

#include <console.h>
#include <hackflight.hpp>
#include <msp.hpp>
#include <task.hpp>
#include <tasks/receiver.hpp>
#include <tasks/guestimator.hpp>

class VisualizerTask : public FreeRTOSTask {

    public:

        void begin(EstimatorTask * estimatorTask, ReceiverTask * receiverTask)
        {
            _receiverTask = receiverTask;

            _estimatorTask = estimatorTask;

            FreeRTOSTask::begin(run, "visualizer", this, 2);
        }

    private:

        static void run(void * obj) 
        {
            ((VisualizerTask *)obj)->run();
        }

        EstimatorTask * _estimatorTask;

        ReceiverTask * _receiverTask;

        Msp _msp;

        void run(void)
        {
            while (true) {

                while (Serial.available()) {

                    if (parse(Serial.read())) {

                        while (available()) {

                            Serial.write(read());
                        }
                    }
                }

            }
        }

        float motors[Hackflight::MAX_MOTOR_COUNT];

        bool parse(const uint8_t byte)
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
                        static int16_t phi;
                        static int8_t dir;

                        dir = 
                            dir == 0 ? +1 : 
                            phi == +450 ? -1 :
                            phi == -450 ? +1 :
                            dir;

                        phi += dir;

                        const int16_t angles[3] = {phi, 0, 0};

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

        bool available(void)
        {
            return _msp.available();
        }

        uint8_t read(void)
        {
            return _msp.read();
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
