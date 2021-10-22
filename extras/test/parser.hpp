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
    P_GOT_CRC        // 6

} parser_state_t;

static uint8_t type2size(uint8_t type)
{
    return type == 121 ? 24 : type == 122 ? 12 : 0;
}

static uint8_t getbyte(uint8_t msgtype, uint8_t index)
{
    const float phi = 1.5, theta = -0.6, psi = 2.7;

    static uint8_t _crc;

    uint8_t byte = index == 1 ? (uint8_t)'$'
                 : index == 2 ? (uint8_t)'M'
                 : index == 3 ? (uint8_t)'>'
                 : index == 4 ? type2size(msgtype)
                 : index == 5 ? msgtype
                 : 0;
}

void parse(uint8_t in, bool & avail, uint8_t & out)
{
    static parser_state_t _pstate;
    static uint8_t _size;
    static uint8_t _type;
    static uint8_t _crc;
    static uint8_t _count;
    static uint8_t _index;
  
    // Parser state transition function
    _pstate
        = _pstate == P_IDLE && in == '$' ? P_GOT_DOLLAR
        : _pstate == P_GOT_DOLLAR && in == 'M' ? P_GOT_M
        : _pstate == P_GOT_M && (in == '<' || in == '>') ? P_GOT_DIRECTION 
        : _pstate == P_GOT_DIRECTION ? P_GOT_SIZE
        : _pstate == P_GOT_SIZE ? P_GOT_TYPE
        : _pstate == P_GOT_TYPE && in == _crc ? P_GOT_CRC
        : _pstate == P_GOT_CRC && _index <= _count ? P_GOT_CRC
        : P_IDLE;

    _size = _pstate == P_GOT_SIZE ? in
          : _pstate == P_IDLE ? 0
          : _size;

    _type = _pstate == P_GOT_TYPE ? in
          : _pstate == P_IDLE ? 0
          : _type;

    _crc = _pstate == P_GOT_SIZE || _pstate == P_GOT_TYPE ?  _crc ^ in 
         : _pstate == P_IDLE  ? 0
         : _crc;

    _count = _pstate == P_GOT_TYPE ? 6 + type2size(in)
           : _pstate == P_GOT_CRC ? _count
           : 0;

    _index = _pstate == P_GOT_CRC ? _index + 1
           : _pstate == P_IDLE ? 0
           : _index;

    avail = _pstate == P_GOT_CRC && _index <= _count;

    out = avail ? getbyte(_type, _index) : 0;
}
