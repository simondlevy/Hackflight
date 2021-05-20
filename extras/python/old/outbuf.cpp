/*
* Implemenation of rft::Board::outbuf() for sim
*
* Copyright (C) 2019 Simon D. Levy
*
* MIT License
*/

#include <RoboFirmwareToolkit.hpp>
#include "../MainModule/Utils.hpp"

void rft::Board::outbuf(char * msg)
{
    debugline(msg);
}
