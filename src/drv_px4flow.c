#include "board.h"
#include "mw.h"

#define PX4FLOW_ADDRESS 0x42

bool initPX4Flow(void)
{
    return i2cWrite(PX4FLOW_ADDRESS, 0x00, 0x00);
}

void pollPX4Flow(void)
{
    i2cRead(PX4FLOW_ADDRESS, 0x00, 22, (uint8_t *)&px4flow_frame);
}
