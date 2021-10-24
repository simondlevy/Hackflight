/*
   Parser for serial comms

   MIT License
 */

#pragma once

#include "copilot.h"

#include "stream_receiver.h"
#include "stream_serial.h"

//extern float stream_receiverRoll;
//extern float stream_receiverPitch;
//extern float stream_receiverYaw;

static uint8_t buffer[128];
static uint8_t buffer_index;
static uint8_t buffer_size;

static void addToOutBuf(bool ready, uint8_t byte)
{
    buffer[buffer_size] = ready ? byte : buffer[buffer_size];
    buffer_size = ready ? buffer_size + 1 : buffer_size;
}

static void serialize( uint8_t & buffer_checksum, bool ready, uint8_t byte)
{
    addToOutBuf(ready, byte);
    buffer_checksum = ready ? buffer_checksum ^ byte : buffer_checksum;
}

static void prepareToSerialize(
        uint8_t & buffer_checksum,
        bool ready,
        uint8_t type,
        uint8_t count,
        uint8_t size)
{
    buffer_size = ready ? 0 : buffer_size;
    buffer_checksum = ready ? 0 : buffer_checksum;

    addToOutBuf(ready, '$');
    addToOutBuf(ready, 'M');
    addToOutBuf(ready, '>');
    serialize(buffer_checksum, ready, count*size);
    serialize(buffer_checksum, ready, type);
}

static void completeSend( uint8_t & buffer_checksum, bool ready)
{
    serialize(buffer_checksum, ready, buffer_checksum);
}

static void prepareToSerializeFloats(
        uint8_t & buffer_checksum,
        bool ready,
        uint8_t type,
        uint8_t count)
{
    prepareToSerialize(buffer_checksum, ready, type, count, 4);
}

static void serializeFloat( uint8_t & buffer_checksum, bool ready, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(buffer_checksum, ready, uintval     & 0xFF);
    serialize(buffer_checksum, ready, uintval>>8  & 0xFF);
    serialize(buffer_checksum, ready, uintval>>16 & 0xFF);
    serialize(buffer_checksum, ready, uintval>>24 & 0xFF);
}

void parse(
        float state_phi,
        float state_theta,
        float state_psi,
        bool & data_available,
        uint8_t & data_byte,
        uint8_t & motor_index,
        uint8_t & motor_percent)
{
    static uint8_t parser_state_;
    static uint8_t type_;
    static uint8_t crc_;
    static uint8_t size_;
    static uint8_t index_;
    static uint8_t buffer_checksum_;

    motor_index = 0;
    motor_percent = 0;

    // Payload functions
    size_ = parser_state_ == 3 ? stream_serialByte : size_;
    index_ = parser_state_ == 5 ? index_ + 1 : 0;
    bool in_payload = type_ >= 200 && parser_state_ == 5 && index_ <= size_;

    // Command acquisition function
    type_ = parser_state_ == 4 ? stream_serialByte : type_;

    // Checksum transition function
    crc_ = parser_state_ == 3 ? stream_serialByte
        : parser_state_ == 4  ?  crc_ ^ stream_serialByte 
        : in_payload ?  crc_ ^ stream_serialByte
        : parser_state_ == 5  ?  crc_
        : 0;

    // Parser state transition function
    parser_state_
        = parser_state_ == 0 && stream_serialByte == '$' ? 1
        : parser_state_ == 1 && stream_serialByte == 'M' ? 2
        : parser_state_ == 2 && (stream_serialByte == '<' || stream_serialByte == '>') ? 3
        : parser_state_ == 3 ? 4
        : parser_state_ == 4 ? 5
        : parser_state_ == 5 && in_payload ? 5
        : parser_state_ == 5 ? 0
        : parser_state_;

    // Incoming payload accumulation
    uint8_t pindex = in_payload ? index_ - 1 : 0;
    buffer[pindex] = in_payload ? stream_serialByte : buffer[pindex];

    // Message dispatch
    bool ready = stream_serialAvailable && parser_state_ == 0 && crc_ == stream_serialByte;
    buffer_index = ready ? 0 : buffer_index;

    switch (type_) {

        case 121:
            {
                prepareToSerializeFloats(buffer_checksum_, ready, type_, 6);
                serializeFloat(buffer_checksum_, ready, stream_receiverThrottle);
                serializeFloat(buffer_checksum_, ready, stream_receiverRoll);
                serializeFloat(buffer_checksum_, ready, stream_receiverPitch);
                serializeFloat(buffer_checksum_, ready, stream_receiverYaw);
                serializeFloat(buffer_checksum_, ready, stream_receiverAux1);
                serializeFloat(buffer_checksum_, ready, stream_receiverAux2);
                completeSend(buffer_checksum_, ready);

            } break;

        case 122:
            {
                prepareToSerializeFloats(buffer_checksum_, ready, type_, 3);
                serializeFloat(buffer_checksum_, ready, state_phi);
                serializeFloat(buffer_checksum_, ready, state_theta);
                serializeFloat(buffer_checksum_, ready, state_psi);
                completeSend(buffer_checksum_, ready);

            } break;

        case 215:
            {
                motor_index = buffer[0];
                motor_percent = buffer[1];

            } break;

    } // switch (type)

    data_available = buffer_size > 0;
    if (data_available) {
        buffer_size--;
        data_byte = buffer[buffer_index];
        buffer_index++;
    }
}
