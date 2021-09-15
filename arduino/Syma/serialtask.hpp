/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include "copilot_extra.h"

#include "mixer.hpp"
#include "mixer.hpp"
#include "state.hpp"
#include "receiver.hpp"

namespace hf {

    class SerialTask {

        friend class Hackflight;

        private:

            uint8_t _outBufChecksum;

            State * _state = NULL;
            Mixer * _mixer = NULL;

            void serialize16(int16_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
            }

            void serialize32(uint32_t a)
            {
                serialize8(a & 0xFF);
                serialize8((a >> 8) & 0xFF);
                serialize8((a >> 16) & 0xFF);
                serialize8((a >> 24) & 0xFF);
            }

            void prepareToSend(uint8_t type, uint8_t count, uint8_t size)
            {
                _outBufChecksum = 0;

                copilot_serialWrite('$');
                copilot_serialWrite('M');
                copilot_serialWrite('>');
                serialize8(count*size);
                serialize8(type);
            }

            void handle_RECEIVER_Request(
                    float & c1,
                    float & c2,
                    float & c3,
                    float & c4,
                    float & c5,
                    float & c6)
            {
                c1 = copilot_receiverThrottle;
                c2 = copilot_receiverRoll;
                c3 = copilot_receiverPitch;
                c4 = copilot_receiverYaw;
                c5 = copilot_receiverAux1;
                c6 = 0;
            }

            void handle_STATE_Request(
                    float & x,
                    float & dx,
                    float & y,
                    float & dy,
                    float & z,
                    float & dz,
                    float & phi,
                    float & dphi,
                    float & theta,
                    float & dtheta,
                    float & psi,
                    float & dpsi)
            {
                x = _state->x[State::X];
                dx = _state->x[State::DX];
                y = _state->x[State::Y];
                dy = _state->x[State::DY];
                z = _state->x[State::Z];
                dz = _state->x[State::DZ];
                phi = _state->x[State::PHI];
                dphi = _state->x[State::DPHI];
                theta = _state->x[State::THETA];
                dtheta = _state->x[State::DTHETA];
                psi = _state->x[State::PSI];
                dpsi = _state->x[State::DPSI];
            }

            void handle_ACTUATOR_TYPE_Request(uint8_t & mtype)
            {
                mtype = _mixer->getType();
            }

            void handle_SET_MOTOR(float  m1, float  m2, float  m3, float  m4)
            {
                _mixer->setMotorDisarmed(0, m1);
                _mixer->setMotorDisarmed(1, m2);
                _mixer->setMotorDisarmed(2, m3);
                _mixer->setMotorDisarmed(3, m4);
            }

        protected:

            static constexpr float FREQ = 66;

            void completeSend(void)
            {
                serialize8(_outBufChecksum);
            }

            void serialize8(uint8_t a)
            {
                copilot_serialWrite(a);
                _outBufChecksum ^= a;
            }

            void prepareToSendBytes(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 1);
            }

            void sendByte(uint8_t src)
            {
                serialize8(src);
            }

            void prepareToSendShorts(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 2);
            }

            void sendFloat(float src)
            {
                copilot_convertFloat(src);
                serialize32(copilot_32bits);
            }

            void prepareToSendInts(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 4);
            }

            void prepareToSendFloats(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 4);
            }

            void begin(void)
            {
                _outBufChecksum = 0;
            }

            void parse(uint8_t c)
            {
                enum {
                    IDLE,
                    GOT_START,
                    GOT_M,
                    GOT_ARROW,
                    GOT_SIZE,
                    IN_PAYLOAD
                }; 

                static uint8_t parser_state;

                static uint8_t type;
                static uint8_t crc;
                static uint8_t size;
                static uint8_t index;

                // Payload functions
                size = parser_state == GOT_ARROW ? c : size;
                index = parser_state == IN_PAYLOAD ? index + 1 : 0;
                bool incoming = type >= 200;
                bool in_payload = incoming && parser_state == IN_PAYLOAD;

                // Command acquisition function
                type = parser_state == GOT_SIZE ? c : type;

                // Checksum transition function
                crc = parser_state == GOT_ARROW ? c
                    : parser_state == IN_PAYLOAD  ?  crc ^ c 
                    : 0;

                // Parser state transition function
                parser_state
                    = parser_state == IDLE && c == '$' ? GOT_START
                    : parser_state == GOT_START && c == 'M' ? GOT_M
                    : parser_state == GOT_M && (c == '<' || c == '>') ? GOT_ARROW
                    : parser_state == GOT_ARROW ? GOT_SIZE
                    : parser_state == GOT_SIZE ? IN_PAYLOAD
                    : parser_state == IN_PAYLOAD && index < size ? IN_PAYLOAD
                    : parser_state == IN_PAYLOAD ? IDLE
                    : parser_state;

                // Payload accumulation
                if (in_payload) {
                    copilot_collectSerialInput(index-1, c);
                }

                // Message dispatch
                if (parser_state == IDLE && crc == c) {
                    dispatchMessage(type);
                }

            } // parse

            void update(Mixer * mixer, State * state)
            {
                if (copilot_serialAvailable) {
                    parse(copilot_serialByte);
                }

                // Support motor testing from GCS
                if (!state->armed) {
                    mixer->runDisarmed();
                }
            }

            void dispatchMessage(uint8_t command)
            {
                switch (command) {

                    case 121:
                        {
                            float c1 = 0;
                            float c2 = 0;
                            float c3 = 0;
                            float c4 = 0;
                            float c5 = 0;
                            float c6 = 0;
                            handle_RECEIVER_Request(c1, c2, c3, c4, c5, c6);
                            prepareToSendFloats(command, 6);
                            sendFloat(c1);
                            sendFloat(c2);
                            sendFloat(c3);
                            sendFloat(c4);
                            sendFloat(c5);
                            sendFloat(c6);
                            completeSend();
                        } break;

                    case 122:
                        {
                            float x = 0;
                            float dx = 0;
                            float y = 0;
                            float dy = 0;
                            float z = 0;
                            float dz = 0;
                            float phi = 0;
                            float dphi = 0;
                            float theta = 0;
                            float dtheta = 0;
                            float psi = 0;
                            float dpsi = 0;
                            handle_STATE_Request(x, dx, y, dy, z, dz,
                                    phi, dphi, theta, dtheta, psi, dpsi);
                            prepareToSendFloats(command, 12);
                            sendFloat(x);
                            sendFloat(dx);
                            sendFloat(y);
                            sendFloat(dy);
                            sendFloat(z);
                            sendFloat(dz);
                            sendFloat(phi);
                            sendFloat(dphi);
                            sendFloat(theta);
                            sendFloat(dtheta);
                            sendFloat(psi);
                            sendFloat(dpsi);
                            completeSend();
                        } break;

                    case 123:
                        {
                            uint8_t mtype = 0;
                            handle_ACTUATOR_TYPE_Request(mtype);
                            prepareToSendBytes(command, 1);
                            sendByte(mtype);
                            completeSend();
                        } break;

                    case 215:
                        {
                            float m1 = copilot_getFloatFromSerialInput(0);
                            float m2 = copilot_getFloatFromSerialInput(4);
                            float m3 = copilot_getFloatFromSerialInput(8);
                            float m4 = copilot_getFloatFromSerialInput(12);

                            handle_SET_MOTOR(m1, m2, m3, m4);

                        } break;

                } // switch (_command)

            } // dispatchMessage 

            void init( Mixer * mixer, State * state)
            {
                _mixer = mixer;
                _state = state;
            }

     }; // class SerialTask

} // namespace hf
