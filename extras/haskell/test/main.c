#include <stdio.h>
#include <stdint.h>

#include "test.h"

void display(uint8_t display_arg0)
{
    printf("%d\n", display_arg0);
}

int main(int argc, char ** argv)
{
    step();

    return 0;
}
