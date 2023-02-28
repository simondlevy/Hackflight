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

#include "logic/core/motors.h"
#include "logic/imu.h"
#include "logic/msp.h"
#include "logic/task/receiver.h"
#include "logic/task/skyranger.h"

class VisualizerTask : public Task {

    private:

        static float scale(const float value)
        {
            return 1000 + 1000 * value;
        }

        // Initialized in constructor
        VehicleState * m_vstate;
        SkyrangerTask *  m_skyrangerTask;

        // Initialized in begin()
        Msp *          m_msp;
        ReceiverTask * m_receiverTask;

        bool m_gotRebootRequest;

        void serializeShorts(
                const uint8_t messageType, const int16_t src[], const uint8_t count)
        {
            m_msp->serializeShorts(messageType, src, count);
        }

        void  readAndConvertMotor(const uint8_t index)
        {
            motors[index] = (m_msp->parseShort(index) - 1000) / 1000.;
        }

    public:

        bool parse(const uint8_t byte)
        {
            if (m_msp->isIdle() && byte == 'R') {
                m_gotRebootRequest = true;
            }

            switch (m_msp->parse(byte)) {

                case 105: // RC
                    {
                        int16_t channels[] = {
                            (int16_t)m_receiverTask->getRawThrottle(),
                            (int16_t)m_receiverTask->getRawRoll(),
                            (int16_t)m_receiverTask->getRawPitch(),
                            (int16_t)m_receiverTask->getRawYaw(),
                            (int16_t)m_receiverTask->getRawAux1(),
                            (int16_t)m_receiverTask->getRawAux2()
                        };

                        serializeShorts(105, channels, 6);

                    } 
                    return true;

                case 108: // ATTITUDE
                    {
                        int16_t angles[3] = {};
                        Imu::getEulerAngles(m_vstate, angles);
                        serializeShorts(108, angles, 3);
                    } 
                    return true;

                case 121: // VL53L5 ranging camera
                    serializeShorts(121, m_skyrangerTask->rangerData, 16);
                    return true;

                case 122: // PAA3905 mocap
                    serializeShorts(122, m_skyrangerTask->mocapData, 2);
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

        VisualizerTask(
                Msp & msp,
                VehicleState & vstate,
                ReceiverTask & receiverTask,
                SkyrangerTask & skyrangerTask) 
            : Task(VISUALIZER, 100) // Hz
        { 
            m_msp = &msp;
            m_vstate = &vstate;
            m_receiverTask = &receiverTask;
            m_skyrangerTask = & skyrangerTask;
        }

        float motors[Motors::MAX_SUPPORTED];

        void begin(ReceiverTask * receiverTask)
        {
            m_receiverTask = receiverTask;
        }

        bool gotRebootRequest(void)
        {
            return m_gotRebootRequest;
        }

}; // class VisualizerTask
