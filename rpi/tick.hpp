#include <stdio.h>
#include <time.h>

static void tick()
{

    time_t now = time(0);
    struct tm * tm = localtime (&now);
    auto time_curr = tm->tm_min * 60 + tm->tm_sec;

    static int evals;

    evals++;

    static int time_prev;

    if (time_curr - time_prev >= 1) {
        if (time_prev > 0) {
            printf("evals/sec = %d\n", evals);
            fflush(stdout);
            evals = 0;
        }
        time_prev = time_curr;
    }
}
