#include "board.h"
#include "mw.h"

#define PX4FLOW_ADDRESS 0x42

uint8_t initPX4Flow()
{
    return i2cWrite(PX4FLOW_ADDRESS, 0x00, 0x00);
}

void pollPX4Flow()
{

    i2cRead(PX4FLOW_ADDRESS, 0x00, 22, (uint8_t *)&px4flow_frame);
}
