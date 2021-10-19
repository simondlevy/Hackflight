/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include "motors.hpp"

#include "copilot.h"

#include "stream_receiver.h"
#include "stream_serial.h"

class Parser {

    private:

        typedef struct {

            uint8_t checksum;
            uint8_t values[128];
            uint8_t index;
            uint8_t size;

        } serial_buffer_t;

        serial_buffer_t _outbuf = {};

        void dispatchMessage(bool ready, uint8_t type, uint8_t * payload, float phi, float theta, float psi, motors_t & motors)
        {
            switch (type) {

                case 121:
                    {
                        prepareToSerializeFloats(ready, type, 6);
                        serializeFloat(ready, stream_receiverThrottle);
                        serializeFloat(ready, stream_receiverRoll);
                        serializeFloat(ready, stream_receiverPitch);
                        serializeFloat(ready, stream_receiverYaw);
                        serializeFloat(ready, stream_receiverAux1);
                        serializeFloat(ready,stream_receiverAux2);
                        completeSend(ready);
                    } break;

                case 122:
                    {
                        prepareToSerializeFloats(ready, type, 12);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, phi);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, theta);
                        serializeFloat(ready, 0);
                        serializeFloat(ready, psi);
                        serializeFloat(ready, 0);
                        completeSend(ready);
                    } break;

                case 215:
                    {
                        uint8_t index = payload[0];
                        uint8_t percent = payload[1];

                        motors.values[0] = ready ? 0 : motors.values[0];
                        motors.values[1] = ready ? 0 : motors.values[1];
                        motors.values[2] = ready ? 0 : motors.values[2];
                        motors.values[3] = ready ? 0 : motors.values[3];

                        uint8_t mindex = ready ? index - 1 : 0;

                        motors.values[mindex] = ready ? percent / 100. : motors.values[mindex];

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

    public:

        bool available(void)
        {
            return _outbuf.size > 0;
        }

        uint8_t read(void)
        {
            _outbuf.size--;
            return _outbuf.values[_outbuf.index++];
        }

        void parse(float phi, float theta, float psi, motors_t & motors)
        {
            uint8_t c = stream_serialByte;

            static uint8_t parser_state;
            static uint8_t payload[128];
            static uint8_t type;
            static uint8_t crc;
            static uint8_t size;
            static uint8_t index;

            // Reset motors iff serial data available
            motors.running = stream_serialAvailable ? true : motors.running;
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
            dispatchMessage(ready, type, payload, phi, theta, psi, motors);

        } // parse

}; // class Parser
