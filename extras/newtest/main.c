#include <stdint.h>
#include <stdio.h>

#include "copilot.h"

void report(uint8_t byte)
{
    printf("%d\n", byte);
}

int main(int argc, char ** argv)
{
    step();

    return 0;
}
