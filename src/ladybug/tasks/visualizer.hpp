/*
   Copyright (c) 2022 Simon D. Levy

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
#include <ladybug/tasks/receiver.hpp>

class VisualizerTask : public LadybugTask {

    private:

        static float scale(const float value)
        {
            return 1000 + 1000 * value;
        }

        void serializeShorts(
                Msp & msp,
                const uint8_t messageType,
                const int16_t src[],
                const uint8_t count)
        {
            msp.serializeShorts(messageType, src, count);
        }

        void  readAndConvertMotor(Msp & msp, const uint8_t index)
        {
            motors[index] = (msp.parseShort(index) - 1000) / 1000.;
        }

    public:

        bool parse(
                vehicleState_t & state,
                ReceiverTask & receiverTask,
                Msp & msp,
                const uint8_t byte)
        {
            switch (msp.parse(byte)) {

                case 105: // RC
                    {
                        int16_t channels[] = {
                            (int16_t)receiverTask.getRawThrottle(),
                            (int16_t)receiverTask.getRawRoll(),
                            (int16_t)receiverTask.getRawPitch(),
                            (int16_t)receiverTask.getRawYaw(),
                            (int16_t)receiverTask.getRawAux1(),
                            (int16_t)receiverTask.getRawAux2()
                        };

                        serializeShorts(msp, 105, channels, 6);

                    } 
                    return true;

                case 108: // ESTIMATOR
                    {
                        const int16_t angles[3] = {
                            (int16_t)(10 * state.phi),
                            (int16_t)(10 * state.theta),
                            (int16_t)state.psi
                        };

                        serializeShorts(msp, 108, angles, 3);
                    } 
                    return true;

                case 214: // SET_MOTORS
                    {
                        readAndConvertMotor(msp, 0);
                        readAndConvertMotor(msp, 1);
                        readAndConvertMotor(msp, 2);
                        readAndConvertMotor(msp, 3);
                    } 
                    break;

                default:
                    break;
            }

            return false;
        }

        VisualizerTask(void)
            : LadybugTask(VISUALIZER, Clock::RATE_100_HZ) // Hz
        { 
        }

        float motors[Hackflight::MAX_MOTOR_COUNT];


}; // class VisualizerTask
