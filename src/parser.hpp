/*
   Parser for serial comms

   MIT License
 */

#pragma once

#include "copilot.h"

#include "stream_receiver.h"
#include "stream_serial.h"

#include "debugger.hpp"
#ifndef stream_receiverRoll
extern float stream_receiverRoll;
extern float stream_receiverPitch;
extern float stream_receiverYaw;
#endif

static void setOutBuf(uint8_t * buffer, uint8_t index, bool ready, uint8_t byte)
{
    buffer[index] = ready ? byte : buffer[index];
}

static void serialize(uint8_t * buffer, uint8_t & buffer_size, uint8_t & crc_out, bool ready, uint8_t byte)
{
    buffer[buffer_size] = ready ? byte : buffer[buffer_size];
    buffer_size = ready ? buffer_size + 1 : buffer_size;
    crc_out = ready ? crc_out ^ byte : crc_out;
}

static void serialize(uint8_t * buffer, uint8_t & buffer_size, bool ready, uint8_t byte)
{
    buffer[buffer_size] = ready ? byte : buffer[buffer_size];
    buffer_size = ready ? buffer_size + 1 : buffer_size;
}

static void serializeFloat(uint8_t * buffer, uint8_t & buffer_size, uint8_t & crc_out, bool ready, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(buffer, buffer_size, crc_out, ready, uintval     & 0xFF);
    serialize(buffer, buffer_size, crc_out, ready, uintval>>8  & 0xFF);
    serialize(buffer, buffer_size, crc_out, ready, uintval>>16 & 0xFF);
    serialize(buffer, buffer_size, crc_out, ready, uintval>>24 & 0xFF);
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
    static uint8_t _buffer[128];
    static uint8_t _buffer_index;
    static uint8_t _buffer_size;

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
    _buffer[pindex] = in_payload ? stream_serialByte : _buffer[pindex];

    // Message dispatch
    bool ready = stream_serialAvailable
              && _parser_state == 0
              && _crc_in == stream_serialByte
              && (_msgtype == 121 || _msgtype == 122);

    _buffer_index = ready ? 0 : _buffer_index;

    uint8_t outsize = 4 * (_msgtype == 121 ? 6 : _msgtype == 122 ? 3 : 0);

    setOutBuf(_buffer, 0, ready, '$');
    setOutBuf(_buffer, 1, ready, 'M');
    setOutBuf(_buffer, 2, ready, '>');

    _buffer_size = ready ? 3 : _buffer_size;
    _buffer[_buffer_size] = ready ? outsize : _buffer[_buffer_size];
    _buffer_size = ready ? 4 : _buffer_size;
    _buffer[_buffer_size] = ready ? _msgtype : _buffer[_buffer_size];
    _buffer_size = ready ? 5 : _buffer_size;

    _crc_out = ready ? outsize ^ _msgtype : 0;

    serializeFloat(_buffer, _buffer_size, _crc_out, ready,
            _msgtype == 121 ? stream_receiverThrottle : _msgtype == 122 ? state_phi : 0);

    serializeFloat(_buffer, _buffer_size, _crc_out, ready,
            _msgtype == 121 ? stream_receiverRoll : _msgtype == 122 ? state_theta : 0);

    serializeFloat(_buffer, _buffer_size, _crc_out, ready,
            _msgtype == 121 ? stream_receiverPitch : _msgtype == 122 ? state_psi : 0);

    bool sendrc = ready && _msgtype == 121;

    serializeFloat(_buffer, _buffer_size, _crc_out, sendrc, stream_receiverYaw);

    serializeFloat(_buffer, _buffer_size, _crc_out, sendrc, stream_receiverAux1);

    serializeFloat(_buffer, _buffer_size, _crc_out, sendrc, stream_receiverAux2);

    serialize(_buffer, _buffer_size, _crc_out, ready, _crc_out);

    motor_index = _msgtype == 215 ? _buffer[0] : 0;
    motor_percent = _msgtype == 215 ? _buffer[1] : 0;

    data_available = _buffer_size > 0;
    if (data_available) {
        _buffer_size--;
        data_byte = _buffer[_buffer_index];
        _buffer_index++;
    }
}
