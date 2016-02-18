#include <stdint.h>
#include <stdbool.h>

#include "../mockduino/drv_i2c.h"

#include "drv_px4flow.h"

#define PX4FLOW_ADDRESS 0x42

px4flow_frame_t px4flow_frame;

bool initPX4Flow(void)
{
    return i2cWrite(PX4FLOW_ADDRESS, 0x00, 0x00);
}

void pollPX4Flow(void)
{
    i2cRead(PX4FLOW_ADDRESS, 0x00, 22, (uint8_t *)&px4flow_frame);
}
