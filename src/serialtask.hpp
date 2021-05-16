/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include <RFT_timertask.hpp>
#include <RFT_board.hpp>
#include <RFT_debugger.hpp>
#include <RFT_actuator.hpp>
#include <RFT_parser.hpp>
#include <rft_timertasks/serialtask.hpp>

#include "actuators/mixer.hpp"

namespace hf {


    class SerialTask : public rft::SerialTask {

        friend class Hackflight;

        private:

        void handle_RECEIVER_Request(float & c1, float & c2, float & c3, float & c4, float & c5, float & c6)
        {
            Receiver * receiver = (Receiver *)_olc;

            c1 = receiver->getRawval(0);
            c2 = receiver->getRawval(1);
            c3 = receiver->getRawval(2);
            c4 = receiver->getRawval(3);
            c5 = receiver->getRawval(4);
            c6 = receiver->getRawval(5);
        }

        void handle_ATTITUDE_RADIANS_Request(float & roll, float & pitch, float & yaw)
        {
            // Cast rft::State to hf::State
            State * state = (State *)_state;

            roll  = state->x[State::PHI];
            pitch = state->x[State::THETA];
            yaw   = state->x[State::PSI];
        }

        void handle_ACTUATOR_TYPE_Request(uint8_t & type)
        {
            type = _actuator->getType();
        }

        void handle_SET_MOTOR_NORMAL(float  m1, float  m2, float  m3, float  m4)
        {
            _actuator->setMotorDisarmed(0, m1);
            _actuator->setMotorDisarmed(1, m2);
            _actuator->setMotorDisarmed(2, m3);
            _actuator->setMotorDisarmed(3, m4);
        }

        protected:

        void dispatchMessage(void) override
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
                        handle_RECEIVER_Request(c1, c2, c3, c4, c5, c6);
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

                    case 123:
                        {
                            uint8_t type = 0;
                            handle_ACTUATOR_TYPE_Request(type);
                            prepareToSendBytes(1);
                            sendByte(type);
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

            } // switch (_command)

        } // dispatchMessage 

    }; // class SerialTask

} // namespace hf
