/*
   Serial comms with parser

   MIT License
 */

#pragma once

#include "../copilot_extra.h"

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

        protected:

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

            void parse(uint8_t c, serial_t & serialByte)
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

                serialByte.status = 0; // assuming nothing

                // Payload accumulation
                if (in_payload) {
                    serialByte.status = 1;
                    serialByte.value = c;
                }

                // Message dispatch
                if (parser_state == IDLE && crc == c) {
                    if (size > 0) {
                        handleSerialInput(type);
                    }
                    else {
                        sendSerialOutput(type);
                    }
                }

            } // parse

            void update(Mixer * mixer, State * state, float * motorsOut, serial_t & serialByte)
            {
                if (copilot_serialAvailable) {
                    parse(copilot_serialByte, serialByte);
                }

                // Support motor testing from GCS
                if (!state->armed) {
                    mixer->runDisarmed(motorsOut);
                }
            }

            void sendSerialOutput(uint8_t type)
            {
                switch (type) {

                    case 121:
                        {
                            float c1 = copilot_receiverThrottle;
                            float c2 = copilot_receiverRoll;
                            float c3 = copilot_receiverPitch;
                            float c4 = copilot_receiverYaw;
                            float c5 = copilot_receiverAux1;
                            float c6 = 0;
                            prepareToSendFloats(type, 6);
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
                            float x = _state->x[State::X];
                            float dx = _state->x[State::DX];
                            float y = _state->x[State::Y];
                            float dy = _state->x[State::DY];
                            float z = _state->x[State::Z];
                            float dz = _state->x[State::DZ];
                            float phi = _state->x[State::PHI];
                            float dphi = _state->x[State::DPHI];
                            float theta = _state->x[State::THETA];
                            float dtheta = _state->x[State::DTHETA];
                            float psi = _state->x[State::PSI];
                            float dpsi = _state->x[State::DPSI];
                            prepareToSendFloats(type, 12);
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
                            uint8_t mtype = _mixer->getType();
                            prepareToSendBytes(type, 1);
                            sendByte(mtype);
                            completeSend();
                        } break;

                } // switch (type)

            } // sendSerialOutput

            void handleSerialInput(uint8_t type)
            {
                switch (type) {

                    case 215:
                        {
                            float m1 = copilot_Input1;
                            float m2 = copilot_Input2;
                            float m3 = copilot_Input3;
                            float m4 = copilot_Input4;

                            _mixer->setMotorDisarmed(0, m1);
                            _mixer->setMotorDisarmed(1, m2);
                            _mixer->setMotorDisarmed(2, m3);
                            _mixer->setMotorDisarmed(3, m4);

                        } break;

                } // switch (type)

            } // handleSerialInput

            void init( Mixer * mixer, State * state)
            {
                _mixer = mixer;
                _state = state;
            }

    }; // class SerialTask

} // namespace hf
