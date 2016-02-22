#include <stdbool.h>
#include <stdint.h>

#include <breezystm32.h>

#include "../sonar.h"

#define MB1242_ADDRESS 0x70

extern bool check_and_update_timed_task(uint32_t * usec, uint32_t period);

static int32_t distance_cm;

static void adjust_reading() {

    distance_cm = 1.071 * distance_cm + 3.103; // emprically determined
}

static bool attempt_write()
{
    return i2cWrite(MB1242_ADDRESS, 0x00, 0x51);
}

bool sonarInit(void)
{
    return attempt_write() == 1;
}

int32_t sonarPoll(void)
{
    static uint32_t mb1242Time = 0;
    static uint8_t state;

    if (check_and_update_timed_task(&mb1242Time, 10000)) {

        if (state == 0) {
            if (attempt_write())
                state++;
        }
        else if (state == 1) {
            uint8_t bytes[2];
            if (i2cRead(MB1242_ADDRESS, 0x8F, 2, bytes)) {
                distance_cm = (bytes[0] << 8) + bytes[1];  
                adjust_reading();
                state++;
            }
        }
        else {
            state = 0;
        }
    }

    return distance_cm;
}
