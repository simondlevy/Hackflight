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
    float state_phi = 1.5;
    float state_theta = -0.6;
    float state_psi = 2.7;

    const uint8_t msgtype = 122;

    bool avail = 0;
    uint8_t byte = 0;

    bool armed = false;

    float m1 = 0;
    float m2 = 0;
    float m3 = 0;
    float m4 = 0;

    parse('$', avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);       // sentinel byte 1
    parse('M', avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);       // sentinel byte 2
    parse('<', avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);       // msg direction
    parse(0, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);         // msg size
    parse(msgtype, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4); 
    parse(0^msgtype, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4); // CRC

    uint8_t count = 0;

    while (avail) {
        if (count > 4 && count < 17) {
            static int32_t intval;
            uint8_t k = (count - 5) % 4;
            intval |= (byte << k*8);
            if (k == 3) {
                printf("%f\n", intval/1000.-2);
                intval = 0;
            }
        }
        parse(0, avail, byte, armed, state_phi, state_theta, state_psi, m1, m2, m3, m4);
        count++;
    }

    return 0;
}
