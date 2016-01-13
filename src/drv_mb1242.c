#include "board.h"
#include "mw.h"

#define MB1242_ADDRESS 0x70

extern bool check_and_update_timed_task(uint32_t * usec, uint32_t period);

static void adjust_reading() {

    SonarAlt = 1.071 * SonarAlt + 3.103; // emprically determined
}

static bool attempt_write()
{
    return i2cWrite(MB1242_ADDRESS, 0x00, 0x51);
}

bool initSonar()
{
    return attempt_write() == 1;
}

void pollSonar()
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
                SonarAlt = (bytes[0] << 8) + bytes[1];  
                adjust_reading();
                state++;
            }
        }
        else {
            state = 0;
        }
    }
}
