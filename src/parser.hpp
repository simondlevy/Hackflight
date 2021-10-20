/*
   Timer task for serial comms

   MIT License
 */

#pragma once

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

        void dispatchMessage(
                bool ready, uint8_t type, uint8_t * payload, 
                float phi, float theta, float psi, 
                float &m1, float &m2, float &m3, float &m4)
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

                        float value =  payload[1] / 100.;

                        m1 = ready ? (index == 1 ?  value : 0) : m1;
                        m2 = ready ? (index == 2 ?  value : 0) : m2;
                        m3 = ready ? (index == 3 ?  value : 0) : m3;
                        m4 = ready ? (index == 4 ?  value : 0) : m4;

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

        void parse(float phi, float theta, float psi, bool armed,
                float & m1, float &m2, float &m3, float &m4)
        {
            uint8_t c = stream_serialByte;

            static uint8_t parser_state_;
            static uint8_t payload_[128];
            static uint8_t type_;
            static uint8_t crc_;
            static uint8_t size_;
            static uint8_t index_;

            // Payload functions
            size_ = parser_state_ == 3 ? c : size_;
            index_ = parser_state_ == 5 ? index_ + 1 : 0;
            bool in_payload = type_ >= 200 && parser_state_ == 5 && index_ <= size_;

            // Command acquisition function
            type_ = parser_state_ == 4 ? c : type_;

            // Checksum transition function
            crc_ = parser_state_ == 3 ? c
                : parser_state_ == 4  ?  crc_ ^ c 
                : in_payload ?  crc_ ^ c
                : parser_state_ == 5  ?  crc_
                : 0;

            // Parser state transition function
            parser_state_
                = parser_state_ == 0 && c == '$' ? 1
                : parser_state_ == 1 && c == 'M' ? 2
                : parser_state_ == 2 && (c == '<' || c == '>') ? 3
                : parser_state_ == 3 ? 4
                : parser_state_ == 4 ? 5
                : parser_state_ == 5 && in_payload ? 5
                : parser_state_ == 5 ? 0
                : parser_state_;

            // Payload accumulation
            uint8_t pindex = in_payload ? index_ - 1 : 0;
            payload_[pindex] = in_payload ? c : payload_[pindex];

            // Message dispatch
            bool ready = stream_serialAvailable && parser_state_ == 0 && crc_ == c;
            static float m1_;
            static float m2_;
            static float m3_;
            static float m4_;
            dispatchMessage(ready, type_, payload_, phi, theta, psi, m1_, m2_, m3_, m4_);

            // Set motors iff in disarmed mode
            m1 = armed ? m1 : m1_;
            m2 = armed ? m2 : m2_;
            m3 = armed ? m3 : m3_;
            m4 = armed ? m4 : m4_;

        } // parse

}; // class Parser
