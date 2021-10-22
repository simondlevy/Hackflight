/*
   Timer task for serial comms

   MIT License
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

typedef enum {

    P_IDLE,          // 0
    P_GOT_DOLLAR,    // 1
    P_GOT_M,         // 2
    P_GOT_DIRECTION, // 3
    P_GOT_SIZE,      // 4
    P_GOT_TYPE,      // 5
    P_IN_PAYLOAD,    // 6
    P_GOT_CRC        // 7

} parser_state_t;

static uint8_t type2size(uint8_t type)
{
    return type == 121 ? 24 : type == 122 ? 12 : 0;
}

static uint8_t float2byte(float value, uint8_t index)
{
    uint32_t uintval = (uint32_t)(1000 * (value + 2));
    return (uintval >> ((index%4)*8)) & 0xFF;
}

static uint8_t state2byte(uint8_t index)
{
    const float phi = 1.5;
    const float theta = -0.6;
    const float psi = 2.7;

    float value = index < 4 ? phi 
                : index < 8 ? theta
                : psi;

    return float2byte(value, index);
}

static uint8_t rx2byte(uint8_t index)
{
    const float c1 = 0.1;
    const float c2 = 0.2;
    const float c3 = 0.3;
    const float c4 = 0.4;
    const float c5 = 0.5;
    const float c6 = 0.6;

    float value = index < 4  ? c1 
                : index < 8  ? c2
                : index < 12 ? c3
                : index < 16 ? c4
                : index < 20 ? c5
                : c6;

    return float2byte(value, index);
}

static uint8_t val2byte(uint8_t msgtype, uint8_t index)
{
    return  msgtype == 121 ? rx2byte(index)
          : msgtype == 122 ? state2byte(index)
          : 0;
}

static uint8_t getbyte(uint8_t msgtype, uint8_t index, uint8_t count)
{
    static uint8_t _crc;

    uint8_t byte = index == 1 ? (uint8_t)'$'
                 : index == 2 ? (uint8_t)'M'
                 : index == 3 ? (uint8_t)'>'
                 : index == 4 ? type2size(msgtype)
                 : index == 5 ? msgtype
                 : index == count ? _crc
                 : val2byte(msgtype, index-6);

     _crc = index > 3 ? _crc ^ byte : 0;

     return byte;
}

void parse(
        uint8_t in_byte,
        bool & out_avail,
        uint8_t & out)
{
    static parser_state_t _pstate;
    static uint8_t _size;
    static uint8_t _type;
    static uint8_t _crc;
    static uint8_t _count;
    static uint8_t _index;
  
    // Parser state transition function
    _pstate
        = _pstate == P_IDLE && in_byte == '$' ? P_GOT_DOLLAR
        : _pstate == P_GOT_DOLLAR && in_byte == 'M' ? P_GOT_M
        : _pstate == P_GOT_M && (in_byte == '<' || in_byte == '>') ? P_GOT_DIRECTION 
        : _pstate == P_GOT_DIRECTION ? P_GOT_SIZE
        : _pstate == P_GOT_SIZE ? P_GOT_TYPE
        : _pstate == P_GOT_TYPE && _size > 0 ? P_IN_PAYLOAD
        : _pstate == P_GOT_TYPE && in_byte == _crc ? P_GOT_CRC
        : _pstate == P_GOT_CRC && _index <= _count ? P_GOT_CRC
        : P_IDLE;

    _size = _pstate == P_GOT_SIZE ? in_byte
          : _pstate == P_IDLE ? 0
          : _size;

    _type = _pstate == P_GOT_TYPE ? in_byte
          : _pstate == P_IDLE ? 0
          : _type;

    _crc = _pstate == P_GOT_SIZE || _pstate == P_GOT_TYPE ?  _crc ^ in_byte 
         : _pstate == P_IDLE  ? 0
         : _crc;

    _count = _pstate == P_GOT_TYPE ? 6 + type2size(in_byte)
           : _pstate == P_GOT_CRC ? _count
           : 0;

    _index = _pstate == P_GOT_CRC ? _index + 1
           : _pstate == P_IDLE ? 0
           : _index;

    out_avail = _pstate == P_GOT_CRC && _index <= _count;

    out = out_avail ? getbyte(_type, _index, _count) : 0;
}
