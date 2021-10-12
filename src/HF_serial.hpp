/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include "HF_state.hpp"
#include "HF_receiver.hpp"
#include "HF_mixer.hpp"
#include "HF_debugger.hpp"

#include "stream_receiver.h"

namespace hf {

    class SerialTask {

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

             void handle_SET_MOTOR(uint8_t index, uint8_t percent, motors_t & motors)
             {
                 motors.values[0] = 0;
                 motors.values[1] = 0;
                 motors.values[2] = 0;
                 motors.values[3] = 0;

                 motors.values[index-1] = percent / 100.;
             }

             void dispatchMessage(uint8_t command, uint8_t * payload, state_t & state, Mixer * mixer, motors_t & motors)
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
                             prepareToSerializeFloats(command, 6);
                             serializeFloat(c1);
                             serializeFloat(c2);
                             serializeFloat(c3);
                             serializeFloat(c4);
                             serializeFloat(c5);
                             serializeFloat(c6);
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
                                     x, dx, y, dy, z, dz, phi, dphi, theta, dtheta, psi, dpsi);
                             prepareToSerializeFloats(command, 12);
                             serializeFloat(x);
                             serializeFloat(dx);
                             serializeFloat(y);
                             serializeFloat(dy);
                             serializeFloat(z);
                             serializeFloat(dz);
                             serializeFloat(phi);
                             serializeFloat(dphi);
                             serializeFloat(theta);
                             serializeFloat(dtheta);
                             serializeFloat(psi);
                             serializeFloat(dpsi);
                             completeSend();
                         } break;

                     case 123:
                         {
                             uint8_t mtype = 0;
                             handle_ACTUATOR_TYPE_Request(mtype, mixer);
                             prepareToSerializeBytes(command, 1);
                             serialize(mtype);
                             completeSend();
                         } break;

                     case 215:
                         {
                             uint8_t index = payload[0];
                             uint8_t percent = payload[1];

                             handle_SET_MOTOR(index, percent, motors);

                         } break;

                 } // switch (_command)

             } // dispatchMessage 

             void prepareToSerialize(uint8_t type, uint8_t count, uint8_t size)
             {
                 _outbuf.size = 0;
                 _outbuf.index = 0;
                 _outbuf.checksum = 0;

                 addToOutBuf('$');
                 addToOutBuf('M');
                 addToOutBuf('>');
                 serialize(count*size);
                 serialize(type);
             }

             void addToOutBuf(uint8_t a)
             {
                 _outbuf.values[_outbuf.size++] = a;
             }

            void completeSend(void)
            {
                serialize(_outbuf.checksum);
            }

            void serialize(uint8_t a)
            {
                addToOutBuf(a);
                _outbuf.checksum ^= a;
            }

            void prepareToSerializeBytes(uint8_t type, uint8_t count)
            {
                prepareToSerialize(type, count, 1);
            }

            void prepareToSerializeFloats(uint8_t type, uint8_t count)
            {
                prepareToSerialize(type, count, 4);
            }

            void serializeFloat(float value)
            {
                uint32_t uintval = 1000 * (value + 2);

                serialize(uintval & 0xFF);
                serialize((uintval>>8) & 0xFF);
                serialize((uintval>>16) & 0xFF);
                serialize((uintval>>24) & 0xFF);
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

            void parse(uint8_t c, state_t & state, Mixer * mixer, motors_t & motors)
            {
                static uint8_t parser_state;
                static uint8_t payload[128];
                static uint8_t type;
                static uint8_t crc;
                static uint8_t size;
                static uint8_t index;

                motors.ready = true;
                motors.values[0] = 0;
                motors.values[1] = 0;
                motors.values[2] = 0;
                motors.values[3] = 0;

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

                /*
                if (parser_state == 0) {
                    Serial1.println();
                }
                Debugger::printf(&Serial1, "state: %d  c: %3d   crc: %d\n", parser_state, c, crc);
                */

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
                if (in_payload) {
                    // Debugger::printf(&Serial1, "payload[%d] = %d\n", index-1, c);
                    payload[index-1] = c;
                }

                // Message dispatch
                if (parser_state == 0 && crc == c) {
                    // Debugger::printf(&Serial1, "Dispatch: %d\n", type);
                    dispatchMessage(type, payload, state, mixer, motors);
                }

            } // parse

    }; // class SerialTask

} // namespace hf
