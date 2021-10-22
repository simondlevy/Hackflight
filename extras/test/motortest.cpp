/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "parser.hpp"

float stream_statePhi = 1.5;
float stream_stateTheta = -0.6;
float stream_statePsi = 2.7;

float stream_receiverThrottle = 0.1;
float stream_receiverRoll = 0.2;
float stream_receiverPitch = 0.3;
float stream_receiverYaw = 0.4;
float stream_receiverAux1 = 0.5;
float stream_receiverAux2 = 0.6;

int main(int argc, char ** argv)
{
    const uint8_t msgtype = 215;
    const uint8_t msgsize = 2;
    const uint8_t mindex = 1;
    const uint8_t mvalue = 52;

    bool avail = 0;
    uint8_t byte = 0;

    uint8_t crc = msgsize ^ msgtype ^ mindex ^ mvalue;

    parse('$', avail, byte);       // sentinel byte 1
    parse('M', avail, byte);       // sentinel byte 2
    parse('<', avail, byte);       // msg direction
    parse(msgsize, avail, byte);         
    parse(msgtype, avail, byte); 
    parse(mindex, avail, byte);   
    parse(mvalue, avail, byte);
    parse(crc, avail, byte);

    return 0;
}
