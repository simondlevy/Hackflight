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

    uint8_t count = 0;

    while (avail) {
        if (count > 4 && count < 17) {
            static uint32_t word;
            uint8_t k = (count - 5) % 4;
            printf("%02d: 0x%02X\n", k, byte);
            word |= (byte << k*8);
            if (k == 3) {
                printf("0x%08X\n", word);
                word = 0;
            }
        }
        parse(0, avail, byte);
        count++;
    }

    return 0;
}
