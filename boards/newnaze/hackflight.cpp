#include <Arduino.h>

#include <hackflight.hpp>
#include "naze.hpp"

hf::Hackflight h;

hf::Board * board;

void setup(void)
{
    Serial.begin(115200);
    board = new hf::Naze();
    h.init(board);
}

void loop(void)
{
    int16_t accelADC[3];
    int16_t gyroADC[3];

    board->imuRead(gyroADC, accelADC);

    Serial.printf("%5d %5d %5d  %5d %5d %5d\n",
            accelADC[0], accelADC[1], accelADC[2], 
            gyroADC[0], gyroADC[1], gyroADC[2]);
}
