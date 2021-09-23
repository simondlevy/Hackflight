#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "timer.h"

void tick(void)
{
    static uint32_t t;
    printf("%04d\n", t++);
}

float copilot_time_sec;

static double get_time(void)
{
    struct timespec ts = {};
    timespec_get(&ts, TIME_UTC);
    return (double)(ts.tv_sec + ts.tv_nsec/1e9);
}

int main(int argc, char ** argv)
{
    float start = get_time();

    for ( ; ;) {

        // copilot_time_sec = get_time() - start;

        printf("%f\n", get_time());

        //step();
    }

    return 0;
} 
