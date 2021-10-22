/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>

#include "parser.hpp"

int main(int argc, char ** argv)
{
    while (true) {

        uint8_t byte = 0;

        printf(" > ");

        scanf("%d", (int *)&byte);

        if (byte == 0) {
            exit(0);
        }

        parse(byte);
    }

    return 0;
}
