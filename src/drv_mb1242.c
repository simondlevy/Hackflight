#include "board.h"
#include "mw.h"

#define MB1242_ADDRESS 0x70

extern bool check_and_update_timed_task(uint32_t * usec, uint32_t period);

static uint8_t attempt_write()
{
    return i2cWrite(MB1242_ADDRESS, 0x00, 0x51);
}

uint8_t initMB1242()
{
    return attempt_write();
}

void pollMB1242()
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
                sonarAlt = (bytes[0] << 8) + bytes[1];  
                state++;
            }
        }
        else {
            state = 0;
        }
    }
}
