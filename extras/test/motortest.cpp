/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "parser.hpp"

float stream_receiverThrottle = 0;
float stream_receiverRoll = 0;
float stream_receiverPitch = 0;
float stream_receiverYaw = 0;
float stream_receiverAux1 = 0;
float stream_receiverAux2 = 0;

int main(int argc, char ** argv)
{
    float state_phi = 0;
    float state_theta = 0;
    float state_psi = 0;

    const uint8_t msgtype = 215;
    const uint8_t msgsize = 2;
    const uint8_t mindex = 1;
    const uint8_t mvalue = 52;

    bool avail = 0;
    uint8_t byte = 0;

    bool armed = false;

    float m1 = 0;
    float m2 = 0;
    float m3 = 0;
    float m4 = 0;

    //              2        215        1        52 
    uint8_t crc = msgsize ^ msgtype ^ mindex ^ mvalue;

    printf("CRC = %d\n", crc);

    parse('$', avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);       // sentinel byte 1
    parse('M', avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);       // sentinel byte 2
    parse('<', avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);       // msg direction
    parse(msgsize, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);         
    parse(msgtype, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4); 
    parse(mindex, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);   
    parse(mvalue, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);
    parse(crc, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);

    return 0;
}
