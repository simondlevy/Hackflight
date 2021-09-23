#include <stdio.h>
#include <stdint.h>

#include "timer.h"

void tick(void)
{
    static uint32_t t;
    printf("%04d\n", t++);
}

int main(int argc, char ** argv)
{
    for ( ; ;) {
        step();
    }

    return 0;
} 
