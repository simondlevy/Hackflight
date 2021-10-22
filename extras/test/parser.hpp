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

static uint8_t type2count(uint8_t type)
{
    return 6 + (type == 121 ? 24 : type == 122 ? 12 : 0);
}

static uint8_t msgbyte(uint8_t msgtype, uint8_t count)
{
    static float phi = 1.5, theta = -0.6, psi = 2.7;

    return 0;
}

void parse(uint8_t in, bool & avail, uint8_t & out)
{
    static parser_state_t pstate_;
    static uint8_t size_;
    static uint8_t type_;
    static uint8_t crc_;
    static uint8_t count_;
  
    // Parser state transition function
    pstate_
        = pstate_ == P_IDLE && in == '$' ? P_GOT_DOLLAR
        : pstate_ == P_GOT_DOLLAR && in == 'M' ? P_GOT_M
        : pstate_ == P_GOT_M && (in == '<' || in == '>') ? P_GOT_DIRECTION 
        : pstate_ == P_GOT_DIRECTION ? P_GOT_SIZE
        : pstate_ == P_GOT_SIZE ? P_GOT_TYPE
        : pstate_ == P_GOT_TYPE && in == crc_ ? P_GOT_CRC
        : pstate_ == P_GOT_CRC && count_ > 0 ? P_GOT_CRC
        : P_IDLE;

    size_ = pstate_ == P_GOT_SIZE ? in : pstate_ == P_IDLE ? 0 : size_;

    type_ = pstate_ == P_GOT_TYPE ? in : pstate_ == P_IDLE ? 0 : type_;

    crc_ = pstate_ == P_GOT_SIZE || pstate_ == P_GOT_TYPE ?  crc_ ^ in 
         : pstate_ == P_IDLE  ? 0
         : crc_;

    count_ = pstate_ == P_GOT_TYPE ? type2count(in) + 2
           : pstate_ == P_GOT_CRC ? count_ - 1
           : 0;

    avail = count_ > 0;
    out = avail ? msgbyte(type_, count_) : 0;
}
