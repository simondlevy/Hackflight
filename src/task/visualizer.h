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

#include "core/motors.h"
#include "esc.h"
#include "imu.h"
#include "msp.h"
#include "receiver.h"
#include "task.h"
#include "task/skyranger.h"

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
        Msp *      m_msp;
        Esc *      m_esc;
        Receiver * m_receiver;

        bool m_gotRebootRequest;

        void sendShorts(
                const uint8_t messageType, const int16_t src[], const uint8_t count)
        {
            m_msp->serializeShorts(messageType, src, count);
            Serial.write(m_msp->payload, m_msp->payloadSize);
        }

        uint8_t parse(const uint8_t byte)
        {
            if (m_msp->isIdle() && byte == 'R') {
                m_gotRebootRequest = true;
            }

            switch (m_msp->parse(byte)) {

                case 105: // RC
                    {
                        int16_t channels[] = {
                            (int16_t)m_receiver->getRawThrottle(),
                            (int16_t)m_receiver->getRawRoll(),
                            (int16_t)m_receiver->getRawPitch(),
                            (int16_t)m_receiver->getRawYaw(),
                            (int16_t)scale(m_receiver->getRawAux1()),
                            (int16_t)scale(m_receiver->getRawAux2())
                        };

                        sendShorts(105, channels, 6);

                    } return 6;

                case 108: // ATTITUDE
                    {
                        int16_t angles[3] = {};
                        Imu::getEulerAngles(m_vstate, angles);
                        sendShorts(108, angles, 3);
                    } return 3;

                case 121: // VL53L5 ranging camera
                    sendShorts(121, m_skyrangerTask->rangerData, 16);
                    return 16;

                case 122: // PAA3905 mocap
                    sendShorts(122, m_skyrangerTask->mocapData, 2);
                    return 2;

                case 214: // SET_MOTORS
                    {
                        motors[0] =
                            m_esc->convertFromExternal(m_msp->parseShort(0));
                        motors[1] =
                            m_esc->convertFromExternal(m_msp->parseShort(1));
                        motors[2] =
                            m_esc->convertFromExternal(m_msp->parseShort(2));
                        motors[3] =
                            m_esc->convertFromExternal(m_msp->parseShort(3));

                    } return 0;

                default:
                    return 0;
            }
        }

    public:

        void run(void)
        {
            while (Serial.available()) {

                parse(Serial.read());
            }
        }

        VisualizerTask(
                Msp & msp,
                VehicleState & vstate,
                SkyrangerTask & skyrangerTask) 
            : Task(VISUALIZER, 100) // Hz
        { 
            m_msp = &msp;
            m_vstate = &vstate;
            m_skyrangerTask = & skyrangerTask;
        }

        float motors[Motors::MAX_SUPPORTED];

        void begin(Esc * esc, Receiver * receiver)
        {
            m_esc = esc;
            m_receiver = receiver;
        }

        bool gotRebootRequest(void)
        {
            return m_gotRebootRequest;
        }

}; // class VisualizerTask
