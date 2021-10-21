/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include "copilot.h"

#include "stream_receiver.h"
#include "stream_serial.h"

typedef struct {

    uint8_t payload[128];
    uint8_t index;

} serial_buffer_t;

static void addToOutBuf(serial_buffer_t & buffer, uint8_t & buffer_size, bool ready, uint8_t a)
{
    buffer.payload[buffer_size] = ready ? a : buffer.payload[buffer_size];
    buffer_size = ready ? buffer_size + 1 : buffer_size;
}

static void serialize(serial_buffer_t & buffer, uint8_t & buffer_size, uint8_t & buffer_checksum, bool ready, uint8_t a)
{
    addToOutBuf(buffer, buffer_size, ready, a);
    buffer_checksum = ready ? buffer_checksum ^ a : buffer_checksum;
}

static void prepareToSerialize(serial_buffer_t & buffer, uint8_t & buffer_size, uint8_t & buffer_checksum, bool ready, uint8_t type, uint8_t count, uint8_t size)
{
    buffer_size = ready ? 0 : buffer_size;
    buffer_checksum = ready ? 0 : buffer_checksum;

    addToOutBuf(buffer, buffer_size, ready, '$');
    addToOutBuf(buffer, buffer_size, ready, 'M');
    addToOutBuf(buffer, buffer_size, ready, '>');
    serialize(buffer, buffer_size, buffer_checksum, ready, count*size);
    serialize(buffer, buffer_size, buffer_checksum, ready, type);
}

static void completeSend(serial_buffer_t & buffer, uint8_t & buffer_size, uint8_t & buffer_checksum, bool ready)
{
    serialize(buffer, buffer_size, buffer_checksum, ready, buffer_checksum);
}

static void prepareToSerializeFloats(serial_buffer_t & buffer, uint8_t & buffer_size, uint8_t & buffer_checksum, bool ready, uint8_t type, uint8_t count)
{
    prepareToSerialize(buffer, buffer_size, buffer_checksum, ready, type, count, 4);
}

static void serializeFloat(serial_buffer_t & buffer, uint8_t & buffer_size, uint8_t & buffer_checksum, bool ready, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(buffer, buffer_size, buffer_checksum, ready, uintval & 0xFF);
    serialize(buffer, buffer_size, buffer_checksum, ready, (buffer, uintval>>8) & 0xFF);
    serialize(buffer, buffer_size, buffer_checksum, ready, (buffer, uintval>>16) & 0xFF);
    serialize(buffer, buffer_size, buffer_checksum, ready, (buffer, uintval>>24) & 0xFF);
}

static void dispatchMessage(
        serial_buffer_t & buffer, uint8_t & buffer_size, uint8_t & buffer_checksum,
        bool ready, uint8_t type,
        float phi, float theta, float psi, 
        float &m1, float &m2, float &m3, float &m4)
{
    switch (type) {

        case 121:
            {
                prepareToSerializeFloats(buffer, buffer_size, buffer_checksum, ready, type, 6);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, stream_receiverThrottle);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, stream_receiverRoll);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, stream_receiverPitch);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, stream_receiverYaw);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, stream_receiverAux1);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready,stream_receiverAux2);
                completeSend(buffer, buffer_size, buffer_checksum, ready);

            } break;

        case 122:
            {
                prepareToSerializeFloats(buffer, buffer_size, buffer_checksum, ready, type, 12);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, phi);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, theta);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, psi);
                serializeFloat(buffer, buffer_size, buffer_checksum, ready, 0);
                completeSend(buffer, buffer_size, buffer_checksum, ready);

            } break;

        case 215:
            {
                uint8_t index = buffer.payload[0];
                float value =  buffer.payload[1] / 100.;

                m1 = ready ? (index == 1 ?  value : 0) : m1;
                m2 = ready ? (index == 2 ?  value : 0) : m2;
                m3 = ready ? (index == 3 ?  value : 0) : m3;
                m4 = ready ? (index == 4 ?  value : 0) : m4;

            } break;

    } // switch (type)

} 


void parser_parse(
        serial_buffer_t & buffer, uint8_t & buffer_size,
        float phi, float theta, float psi, bool armed,
        float & m1, float &m2, float &m3, float &m4)
{
    uint8_t c = stream_serialByte;

    static uint8_t parser_state_;
    static uint8_t type_;
    static uint8_t crc_;
    static uint8_t size_;
    static uint8_t index_;
    static uint8_t buffer_size_;
    static uint8_t buffer_checksum_;

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
    buffer.payload[pindex] = in_payload ? c : buffer.payload[pindex];

    // Message dispatch
    bool ready = stream_serialAvailable && parser_state_ == 0 && crc_ == c;
    buffer.index = ready ? 0 : buffer.index;
    static float m1_;
    static float m2_;
    static float m3_;
    static float m4_;
    dispatchMessage(buffer, buffer_size_, buffer_checksum_, ready, type_, phi, theta, psi, m1_, m2_, m3_, m4_);

    // Set motors iff in disarmed mode
    m1 = armed ? m1 : m1_;
    m2 = armed ? m2 : m2_;
    m3 = armed ? m3 : m3_;
    m4 = armed ? m4 : m4_;

    buffer_size = buffer_size_;
}

uint8_t parser_read(serial_buffer_t & buffer, uint8_t & buffer_size)
{
    buffer_size--;
    uint8_t retval = buffer.payload[buffer.index];
    buffer.index++;
    return retval;
}
