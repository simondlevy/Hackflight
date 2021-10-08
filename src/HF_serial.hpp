/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include "HF_timer.hpp"
#include "HF_state.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"

namespace hf {

    class SerialTask {

        friend class HackflightFull;

        private:

            static const uint8_t MAXMSG = 255;

            static const int OUTBUF_SIZE = 128;

            uint8_t _outBufChecksum;
            uint8_t _outBuf[OUTBUF_SIZE];
            uint8_t _outBufIndex;
            uint8_t _outBufSize;

            bool _useTelemetryPort = false;

            Timer timer = Timer(66);

            uint8_t _payload[128] = {};

             void handle_RECEIVER_Request(
                     float & c1,
                     float & c2,
                     float & c3,
                     float & c4,
                     float & c5,
                     float & c6)
            {
                c1 = copilot_receiverThrottle ;
                c2 = copilot_receiverRoll ;
                c3 = copilot_receiverPitch ;
                c4 = copilot_receiverYaw ;
                c5 = copilot_receiverAux1 ;
                c6 = copilot_receiverAux2 ;
             }

             void handle_STATE_Request(
                     State * state,
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
                 x = state->x[State::X];
                 dx = state->x[State::DX];
                 y = state->x[State::Y];
                 dy = state->x[State::DY];
                 z = state->x[State::Z];
                 dz = state->x[State::DZ];
                 phi = state->x[State::PHI];
                 dphi = state->x[State::DPHI];
                 theta = state->x[State::THETA];
                 dtheta = state->x[State::DTHETA];
                 psi = state->x[State::PSI];
                 dpsi = state->x[State::DPSI];
             }

             void handle_ACTUATOR_TYPE_Request(uint8_t & mtype, Mixer * mixer)
             {
                 mtype = mixer->getType();
             }

             void handle_SET_MOTOR(float  m1, float  m2, float  m3, float  m4, Mixer * mixer)
             {
                 mixer->setMotorDisarmed(0, m1);
                 mixer->setMotorDisarmed(1, m2);
                 mixer->setMotorDisarmed(2, m3);
                 mixer->setMotorDisarmed(3, m4);
             }

             void collectPayload(uint8_t index, uint8_t value)
             {
                 _payload[index] = value;
             }

             void dispatchMessage(uint8_t command, State * state, Mixer * mixer)
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
                             handle_STATE_Request(
                                     state,
                                     x,
                                     dx,
                                     y,
                                     dy,
                                     z,
                                     dz,
                                     phi,
                                     dphi,
                                     theta,
                                     dtheta,
                                     psi,
                                     dpsi);
                             prepareToSendFloats(command,
                                     12);
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
                             handle_ACTUATOR_TYPE_Request(mtype, mixer);
                             prepareToSendBytes(command, 1);
                             sendByte(mtype);
                             completeSend();
                         } break;

                     case 215:
                         {
                             float m1 = 0;
                             memcpy(&m1,  &_payload[0], sizeof(float));

                             float m2 = 0;
                             memcpy(&m2,  &_payload[4], sizeof(float));

                             float m3 = 0;
                             memcpy(&m3,  &_payload[8], sizeof(float));

                             float m4 = 0;
                             memcpy(&m4,  &_payload[12], sizeof(float));

                             handle_SET_MOTOR(m1, m2, m3, m4, mixer);
                         } break;

                 } // switch (_command)

             } // dispatchMessage 

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
                 _outBufSize = 0;
                 _outBufIndex = 0;
                 _outBufChecksum = 0;

                 addToOutBuf('$');
                 addToOutBuf('M');
                 addToOutBuf('>');
                 serialize8(count*size);
                 serialize8(type);
             }

             void addToOutBuf(uint8_t a)
             {
                 _outBuf[_outBufSize++] = a;
            }

            void completeSend(void)
            {
                serialize8(_outBufChecksum);
            }

            void serialize8(uint8_t a)
            {
                addToOutBuf(a);
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

            void sendShort(short src)
            {
                int16_t a;
                memcpy(&a, &src, 2);
                serialize16(a);
            }

            void prepareToSendInts(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 4);
            }

            void sendInt(int32_t src)
            {
                int32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            void prepareToSendFloats(uint8_t type, uint8_t count)
            {
                prepareToSend(type, count, 4);
            }

            void sendFloat(float src)
            {
                uint32_t a;
                memcpy(&a, &src, 4);
                serialize32(a);
            }

            void begin(void)
            {
                _outBufChecksum = 0;
                _outBufIndex = 0;
                _outBufSize = 0;
            }

            uint8_t availableBytes(void)
            {
                return _outBufSize;
            }

            uint8_t readByte(void)
            {
                _outBufSize--;
                return _outBuf[_outBufIndex++];
            }

            void parse(uint8_t c, State * state, Mixer * mixer)
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
                    collectPayload(index-1, c);
                }

                // Message dispatch
                if (parser_state == IDLE && crc == c) {
                    dispatchMessage(type, state, mixer);
                }

            } // parse

    protected:

            void update(uint32_t time_usec, State * state, Mixer * mixer, float * motorvals)
            {
                extern bool serialAvailable(void);
                extern uint8_t serialRead(void);
                extern void serialWrite(uint8_t);

                if (!timer.ready(time_usec)) {
                    return;
                }

                while (serialAvailable() > 0) {
                    parse(serialRead(), state, mixer);
                }

                while (availableBytes() > 0) {
                    serialWrite(readByte());
                }

                // Support motor testing from GCS
                if (!state->armed) {
                    mixer->runDisarmed();
                }
            }

    public:

            SerialTask(bool secondaryPort=false)
            {
                _useTelemetryPort = secondaryPort;
            }

        }; // class SerialTask

} // namespace hf
