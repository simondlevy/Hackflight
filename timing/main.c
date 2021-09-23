#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "timer.h"

void tick(void)
{
    static uint32_t t;
    printf("%04d\n", t++);
}

void debug(float t, float p)
{
    printf("%f %f\n", t, p);
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
    double start = get_time();

    for ( ; ;) {

        static float time_sec_prev;

        copilot_time_sec = (float)(get_time() - start);

        /*
        time_sec_prev = (copilot_time_sec - time_sec_prev) > 1 ? copilot_time_sec : time_sec_prev;

        if (copilot_time_sec - time_sec_prev == 0) {
            printf("tick\n");
        }
        */

        step();
    }

    return 0;
} 
