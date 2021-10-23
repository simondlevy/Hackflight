/*
   Parser tester

   MIT License
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "parser.hpp"

float stream_receiverThrottle = 0.1;
float stream_receiverRoll = 0.2;
float stream_receiverPitch = 0.3;
float stream_receiverYaw = 0.4;
float stream_receiverAux1 = 0.5;
float stream_receiverAux2 = 0.6;

int main(int argc, char ** argv)
{
    float state_phi = 0;
    float state_theta = 0;
    float state_psi = 0;

    const uint8_t msgtype = 121;

    bool avail = 0;
    uint8_t byte = 0;

    uint8_t motor_index = 0;
    uint8_t motor_percent = 0;

    parse('$', avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);       // sentinel byte 1
    parse('M', avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);       // sentinel byte 2
    parse('<', avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);       // msg direction
    parse(0, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);         // msg size
    parse(msgtype, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent); 
    parse(0^msgtype, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent); // CRC

    uint8_t count = 0;

    while (avail) {
        if (count > 4 && count < 29) {
            static int32_t intval;
            uint8_t k = (count - 5) % 4;
            intval |= (byte << k*8);
            if (k == 3) {
                printf("%f\n", intval/1000.-2);
                intval = 0;
            }
        }
        parse(0, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);
        count++;
    }

    return 0;
}
