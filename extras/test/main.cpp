/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>

#include "parser.hpp"

int main(int argc, char ** argv)
{
    parse('$');   // sentinel byte 1
    parse('M');   // sentinel byte 2
    parse('<');   // msg direction
    parse(0);     // msg size
    parse(215);   // msg type
    parse(0^215); // CRC

    return 0;
}
