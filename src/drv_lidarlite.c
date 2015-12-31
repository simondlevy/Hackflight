#include "board.h"
#include "mw.h"

#define LIDARLITE_ADDRESS 0x62

extern bool check_and_update_timed_task(uint32_t * usec, uint32_t period);

static uint8_t attempt_write()
{
    return i2cWrite(LIDARLITE_ADDRESS, 0x00, 0x04);
}

uint8_t initLidarLite()
{
    return attempt_write();
}

void pollLidarLite()
{
    static uint32_t lidarTime = 0;
    static uint8_t state;

    if (check_and_update_timed_task(&lidarTime, 10000)) {

        if (state == 0) {
            if (attempt_write())
                state++;
        }
        else if (state == 1) {
            uint8_t bytes[2];
            if (i2cRead(LIDARLITE_ADDRESS, 0x8F, 2, bytes)) {
                lidarlite_distance = (bytes[0] << 8) + bytes[1];  
                state++;
            }
        }
        else {
            state = 0;
        }
    }
}
