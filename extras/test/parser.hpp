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


void parse(uint8_t byte)
{
    static parser_state_t pstate_;

    static float phi = 1.5, theta = -0.6, psi = 2.7;

    // Parser state transition function
    pstate_
        = pstate_ == P_IDLE && byte == '$' ? P_GOT_DOLLAR
        : pstate_ == P_GOT_DOLLAR && byte == 'M' ? P_GOT_M
        : pstate_ == P_GOT_M && (byte == '<' || byte == '>') ? P_GOT_DIRECTION 
        : pstate_ == P_GOT_DIRECTION ? P_GOT_SIZE
        : pstate_ == P_GOT_SIZE ? P_GOT_TYPE
        : pstate_ == P_GOT_TYPE ? P_GOT_CRC
        : P_IDLE;

    static uint8_t size_;
    static uint8_t type_;

    size_ = pstate_ == P_GOT_SIZE ? byte : pstate_ == P_IDLE ? 0 : size_;

    type_ = pstate_ == P_GOT_TYPE ? byte : pstate_ == P_IDLE ? 0 : type_;
    
    if (type_ > 0) printf("%d\n", type_);
}
