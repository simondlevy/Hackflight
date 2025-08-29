#include <pmw3901.hpp>

static const uint8_t CS_PIN = 3;

static PMW3901 _pmw3901;

bool opticalflow_deviceInit()
{
    SPI.begin();

    return _pmw3901.begin(CS_PIN);
}

void opticalflow_deviceRead(int16_t & deltaX, int16_t & deltaY, bool & gotMotion)
{
    _pmw3901.readMotion(deltaX, deltaY, gotMotion);
}
