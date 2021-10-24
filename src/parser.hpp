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
static uint8_t buffer_size;

static void addToOutBuf(bool ready, uint8_t byte)
{
    buffer[buffer_size] = ready ? byte : buffer[buffer_size];
    buffer_size = ready ? buffer_size + 1 : buffer_size;
}

static void serialize( uint8_t & crc_out, bool ready, uint8_t byte)
{
    addToOutBuf(ready, byte);
    crc_out = ready ? crc_out ^ byte : crc_out;
}

static void prepareToSerialize(
        uint8_t & crc_out,
        bool ready,
        uint8_t type,
        uint8_t count,
        uint8_t size)
{
    buffer_size = ready ? 0 : buffer_size;
    crc_out = ready ? 0 : crc_out;

    addToOutBuf(ready, '$');
    addToOutBuf(ready, 'M');
    addToOutBuf(ready, '>');
    serialize(crc_out, ready, count*size);
    serialize(crc_out, ready, type);
}

static void completeSend( uint8_t & crc_out, bool ready)
{
    serialize(crc_out, ready, crc_out);
}

static void prepareToSerializeFloats(
        uint8_t & crc_out,
        bool ready,
        uint8_t type,
        uint8_t count)
{
    prepareToSerialize(crc_out, ready, type, count, 4);
}

static void serializeFloat( uint8_t & crc_out, bool ready, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(crc_out, ready, uintval     & 0xFF);
    serialize(crc_out, ready, uintval>>8  & 0xFF);
    serialize(crc_out, ready, uintval>>16 & 0xFF);
    serialize(crc_out, ready, uintval>>24 & 0xFF);
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
    static uint8_t _parser_state;
    static uint8_t _msgtype;
    static uint8_t _crc_in;
    static uint8_t _input_size;
    static uint8_t _payload_index;
    static uint8_t _crc_out;
    static uint8_t _buffer_index;

    motor_index = 0;
    motor_percent = 0;

    // Payload functions
    _input_size = _parser_state == 3 ? stream_serialByte : _input_size;
    _payload_index = _parser_state == 5 ? _payload_index + 1 : 0;
    bool in_payload = _msgtype >= 200 && _parser_state == 5 && _payload_index <= _input_size;

    // Command acquisition function
    _msgtype = _parser_state == 4 ? stream_serialByte : _msgtype;

    // Checksum transition function
    _crc_in = _parser_state == 3 ? stream_serialByte
        : _parser_state == 4  ?  _crc_in ^ stream_serialByte 
        : in_payload ?  _crc_in ^ stream_serialByte
        : _parser_state == 5  ?  _crc_in
        : 0;

    // Parser state transition function
    _parser_state
        = _parser_state == 0 && stream_serialByte == '$' ? 1
        : _parser_state == 1 && stream_serialByte == 'M' ? 2
        : _parser_state == 2 && (stream_serialByte == '<' || stream_serialByte == '>') ? 3
        : _parser_state == 3 ? 4
        : _parser_state == 4 ? 5
        : _parser_state == 5 && in_payload ? 5
        : _parser_state == 5 ? 0
        : _parser_state;

    // Incoming payload accumulation
    uint8_t pindex = in_payload ? _payload_index - 1 : 0;
    buffer[pindex] = in_payload ? stream_serialByte : buffer[pindex];

    // Message dispatch
    bool ready = stream_serialAvailable && _parser_state == 0 && _crc_in == stream_serialByte;
    _buffer_index = ready ? 0 : _buffer_index;

    switch (_msgtype) {

        case 121:
            {
                prepareToSerializeFloats(_crc_out, ready, _msgtype, 6);
                serializeFloat(_crc_out, ready, stream_receiverThrottle);
                serializeFloat(_crc_out, ready, stream_receiverRoll);
                serializeFloat(_crc_out, ready, stream_receiverPitch);
                serializeFloat(_crc_out, ready, stream_receiverYaw);
                serializeFloat(_crc_out, ready, stream_receiverAux1);
                serializeFloat(_crc_out, ready, stream_receiverAux2);
                completeSend(_crc_out, ready);

            } break;

        case 122:
            {
                prepareToSerializeFloats(_crc_out, ready, _msgtype, 3);
                serializeFloat(_crc_out, ready, state_phi);
                serializeFloat(_crc_out, ready, state_theta);
                serializeFloat(_crc_out, ready, state_psi);
                completeSend(_crc_out, ready);

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
        data_byte = buffer[_buffer_index];
        _buffer_index++;
    }
}
