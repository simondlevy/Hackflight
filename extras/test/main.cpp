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

    parse('$');   // sentinel byte 1
    parse('M');   // sentinel byte 2
    parse('<');   // msg direction
    parse(0);     // msg size
    parse(msgtype); 
    parse(0^msgtype); // CRC

    return 0;
}
