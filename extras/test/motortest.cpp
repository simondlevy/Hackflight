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

    uint8_t motor_index = 0;
    uint8_t motor_percent = 0;

    //              2        215        1        52 
    uint8_t crc = msgsize ^ msgtype ^ mindex ^ mvalue;

    parse('$', avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);       // sentinel byte 1
    printf("%d %d\n", motor_index, motor_percent);

    parse('M', avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);       // sentinel byte 2
    printf("%d %d\n", motor_index, motor_percent);

    parse('<', avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);       // msg direction
    printf("%d %d\n", motor_index, motor_percent);

    parse(msgsize, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);         
    printf("%d %d\n", motor_index, motor_percent);

    parse(msgtype, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent); 
    printf("%d %d\n", motor_index, motor_percent);

    parse(mindex, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);   
    printf("%d %d\n", motor_index, motor_percent);

    parse(mvalue, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);
    printf("%d %d\n", motor_index, motor_percent);

    parse(crc, avail, byte, state_phi, state_theta, state_psi, motor_index, motor_percent);
    printf("%d %d\n", motor_index, motor_percent);

    return 0;
}
