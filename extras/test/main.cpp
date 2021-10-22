/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>

#include "parser.hpp"

int main(int argc, char ** argv)
{
    const uint8_t msgtype = 122;

    bool avail = 0;
    uint8_t byte = 0;

    parse('$', avail, byte);       // sentinel byte 1
    parse('M', avail, byte);       // sentinel byte 2
    parse('<', avail, byte);       // msg direction
    parse(0, avail, byte);         // msg size
    parse(msgtype, avail, byte); 
    parse(0^msgtype, avail, byte); // CRC

    while (avail) {
        parse(0, avail, byte);
        printf("0x%02X\n", byte);
    }

    return 0;
}
