/*
   Timer task for serial comms

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#pragma once

#include <RFT_timertask.hpp>
#include <RFT_board.hpp>
#include <RFT_debugger.hpp>
#include <RFT_actuator.hpp>
#include <RFT_serialtask.hpp>
#include <RFT_parser.hpp>

#include "mavstate.hpp"

namespace hf {

    class SerialTask : public rft::SerialTask {

        friend class Hackflight;

        private:

            void handle_Receiver_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6)
            {
                c1 = _olc->getRawval(0);
                c2 = _olc->getRawval(1);
                c3 = _olc->getRawval(2);
                c4 = _olc->getRawval(3);
                c5 = _olc->getRawval(4);
                c6 = _olc->getRawval(5);
            }

            void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw)
            {
                MavState * mavstate = (MavState *)_state;

                roll  = mavstate->x[MavState::STATE_PHI];
                pitch = mavstate->x[MavState::STATE_THETA];
                yaw   = mavstate->x[MavState::STATE_PSI];
            }

            void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4)
            {
                _actuator->setMotorDisarmed(0, m1);
                _actuator->setMotorDisarmed(1, m2);
                _actuator->setMotorDisarmed(2, m3);
                _actuator->setMotorDisarmed(3, m4);
            }


            // Parser overrides -------------------------------------------------------

            void dispatchMessage(void)
            {
                switch (_command) {

                    case 121:
                        {
                            float c1 = 0;
                            float c2 = 0;
                            float c3 = 0;
                            float c4 = 0;
                            float c5 = 0;
                            float c6 = 0;
                            handle_Receiver_Request(c1, c2, c3, c4, c5, c6);
                            prepareToSendFloats(6);
                            sendFloat(c1);
                            sendFloat(c2);
                            sendFloat(c3);
                            sendFloat(c4);
                            sendFloat(c5);
                            sendFloat(c6);
                            serialize8(_checksum);
                        } break;

                    case 122:
                        {
                            float roll = 0;
                            float pitch = 0;
                            float yaw = 0;
                            handle_ATTITUDE_RADIANS_Request(roll, pitch, yaw);
                            prepareToSendFloats(3);
                            sendFloat(roll);
                            sendFloat(pitch);
                            sendFloat(yaw);
                            serialize8(_checksum);
                        } break;

                    case 215:
                        {
                            float m1 = 0;
                            memcpy(&m1,  &_inBuf[0], sizeof(float));

                            float m2 = 0;
                            memcpy(&m2,  &_inBuf[4], sizeof(float));

                            float m3 = 0;
                            memcpy(&m3,  &_inBuf[8], sizeof(float));

                            float m4 = 0;
                            memcpy(&m4,  &_inBuf[12], sizeof(float));

                            handle_SET_MOTOR_NORMAL(m1, m2, m3, m4);
                        } break;
                }
            }

    };  // SerialTask

} // namespace hf
