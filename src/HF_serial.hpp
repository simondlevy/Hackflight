/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include "HF_state.hpp"
#include "HF_mixer.hpp"
#include "HF_debugger.hpp"

#include "copilot.h"

#include "stream_receiver.h"
#include "stream_serial.h"

namespace hf {

    class SerialComms {

        friend class HackflightFull;

        private:

            typedef struct {

                uint8_t checksum;
                uint8_t values[128];
                uint8_t index;
                uint8_t size;

            } serial_buffer_t;

            serial_buffer_t _outbuf = {};

            void handle_RECEIVER_Request(
                     float & c1,
                     float & c2,
                     float & c3,
                     float & c4,
                     float & c5,
                     float & c6)
            {
                c1 = stream_receiverThrottle;
                c2 = stream_receiverRoll;
                c3 = stream_receiverPitch;
                c4 = stream_receiverYaw;
                c5 = stream_receiverAux1;
                c6 = stream_receiverAux2;
            }

             void handle_STATE_Request(
                     state_t & state,
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
                 x = state.x;
                 dx = state.dx;
                 y = state.y;
                 dy = state.dy;
                 z = state.z;
                 dz = state.dz;
                 phi = state.phi;
                 dphi = state.dphi;
                 theta = state.theta;
                 dtheta = state.dtheta;
                 psi = state.psi;
                 dpsi = state.dpsi;
             }

             void handle_ACTUATOR_TYPE_Request(uint8_t & mtype, Mixer * mixer)
             {
                 mtype = mixer->getType();
             }

             void handle_SET_MOTOR(bool ready, uint8_t index, uint8_t percent, motors_t & motors)
             {
                 motors.values[0] = ready ? 0 : motors.values[0];
                 motors.values[1] = ready ? 0 : motors.values[1];
                 motors.values[2] = ready ? 0 : motors.values[2];
                 motors.values[3] = ready ? 0 : motors.values[3];

                 uint8_t mindex = ready ? index - 1 : 0;
                 motors.values[mindex] = ready ? percent / 100. : motors.values[mindex];
             }

             void dispatchMessage(bool ready, uint8_t type, uint8_t * payload, state_t & state, Mixer * mixer, motors_t & motors)
             {
                 switch (type) {

                     case 121:
                         {
                             float c1 = 0;
                             float c2 = 0;
                             float c3 = 0;
                             float c4 = 0;
                             float c5 = 0;
                             float c6 = 0;
                             handle_RECEIVER_Request(c1, c2, c3, c4, c5, c6);
                             prepareToSerializeFloats(ready, type, 6);
                             serializeFloat(ready, c1);
                             serializeFloat(ready, c2);
                             serializeFloat(ready, c3);
                             serializeFloat(ready, c4);
                             serializeFloat(ready, c5);
                             serializeFloat(ready, c6);
                             completeSend(ready);
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
                                     x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi);
                             prepareToSerializeFloats(ready, type, 12);
                             serializeFloat(ready, x);
                             serializeFloat(ready, dx);
                             serializeFloat(ready, y);
                             serializeFloat(ready, dy);
                             serializeFloat(ready, z);
                             serializeFloat(ready, dz);
                             serializeFloat(ready, phi);
                             serializeFloat(ready, dphi);
                             serializeFloat(ready, theta);
                             serializeFloat(ready, dtheta);
                             serializeFloat(ready, psi);
                             serializeFloat(ready, dpsi);
                             completeSend(ready);
                         } break;

                     case 123:
                         {
                             uint8_t mtype = 0;
                             handle_ACTUATOR_TYPE_Request(mtype, mixer);
                             prepareToSerializeBytes(ready, type, 1);
                             serialize(ready, mtype);
                             completeSend(ready );
                         } break;

                     case 215:
                         {
                             uint8_t index = payload[0];
                             uint8_t percent = payload[1];

                             handle_SET_MOTOR(ready, index, percent, motors);

                         } break;

                 } // switch (type)

             } // dispatchMessage 

             void prepareToSerialize(bool ready, uint8_t type, uint8_t count, uint8_t size)
             {
                 _outbuf.size = ready ? 0 : _outbuf.size;
                 _outbuf.index = ready ? 0 : _outbuf.index;
                 _outbuf.checksum = ready ? 0 : _outbuf.checksum;

                 addToOutBuf(ready, '$');
                 addToOutBuf(ready, 'M');
                 addToOutBuf(ready, '>');
                 serialize(ready, count*size);
                 serialize(ready, type);
             }

             void addToOutBuf(bool ready, uint8_t a)
             {
                 _outbuf.values[_outbuf.size] = ready ? a : _outbuf.values[_outbuf.size];

                 _outbuf.size = ready ? _outbuf.size + 1 : _outbuf.size;
             }

            void completeSend(bool ready)
            {
                serialize(ready, _outbuf.checksum);
            }

            void serialize(bool ready, uint8_t a)
            {
                addToOutBuf(ready, a);
                _outbuf.checksum = ready ? _outbuf.checksum ^ a : _outbuf.checksum;
            }

            void prepareToSerializeBytes(bool ready, uint8_t type, uint8_t count)
            {
                prepareToSerialize(ready, type, count, 1);
            }

            void prepareToSerializeFloats(bool ready, uint8_t type, uint8_t count)
            {
                prepareToSerialize(ready, type, count, 4);
            }

            void serializeFloat(bool ready, float value)
            {
                uint32_t uintval = 1000 * (value + 2);

                serialize(ready, uintval & 0xFF);
                serialize(ready, (uintval>>8) & 0xFF);
                serialize(ready, (uintval>>16) & 0xFF);
                serialize(ready, (uintval>>24) & 0xFF);
            }

        protected:

            uint8_t available(void)
            {
                return _outbuf.size;
            }

            uint8_t read(void)
            {
                _outbuf.size--;
                return _outbuf.values[_outbuf.index++];
            }

            void parse(state_t & state, Mixer * mixer, motors_t & motors)
            {
                uint8_t c = stream_serialByte;

                static uint8_t parser_state;
                static uint8_t payload[128];
                static uint8_t type;
                static uint8_t crc;
                static uint8_t size;
                static uint8_t index;

                // Reset motors iff serial data available
                motors.ready = stream_serialAvailable ? true : motors.ready;
                motors.values[0] = stream_serialAvailable ? 0 : motors.values[0];
                motors.values[1] = stream_serialAvailable ? 0 : motors.values[1];
                motors.values[2] = stream_serialAvailable ? 0 : motors.values[2];
                motors.values[3] = stream_serialAvailable ? 0 : motors.values[3];

                // Payload functions
                size = parser_state == 3 ? c : size;
                index = parser_state == 5 ? index + 1 : 0;
                bool in_payload = type >= 200 && parser_state == 5 && index <= size;

                // Command acquisition function
                type = parser_state == 4 ? c : type;

                // Checksum transition function
                crc = parser_state == 3 ? c
                    : parser_state == 4  ?  crc ^ c 
                    : in_payload ?  crc ^ c
                    : parser_state == 5  ?  crc
                    : 0;

                // Parser state transition function
                parser_state
                    = parser_state == 0 && c == '$' ? 1
                    : parser_state == 1 && c == 'M' ? 2
                    : parser_state == 2 && (c == '<' || c == '>') ? 3
                    : parser_state == 3 ? 4
                    : parser_state == 4 ? 5
                    : parser_state == 5 && in_payload ? 5
                    : parser_state == 5 ? 0
                    : parser_state;

                // Payload accumulation
                uint8_t pindex = in_payload ? index - 1 : 0;
                payload[pindex] = in_payload ? c : payload[pindex];

                // Message dispatch
                bool ready = stream_serialAvailable && parser_state == 0 && crc == c;
                dispatchMessage(ready, type, payload, state, mixer, motors);

            } // parse

    }; // class SerialComms

} // namespace hf
