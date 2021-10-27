/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "copilot.h"
#include "stream_receiver.h"

#include "serial.hpp"
#include "parser.hpp"
#include "debugger.hpp"

void stream_writeBrushedMotors(
        uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin,
        float m1_val, float m2_val, float m3_val, float m4_val);

void setup(void)
{
}

void loop(void)
{
    step();
}

void stream_run(
        float state_phi,
        float state_theta,
        float state_psi,
        bool armed,
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin,
        float m1_flying,
        float m2_flying,
        float m3_flying,
        float m4_flying,
        bool serialAvail,
        uint8_t serialByte)
{
    bool sending = false;
    bool receiving = false;
    uint8_t inbuff_index = 0;
    uint8_t msgtype = 0;

    //parse(serialAvail, serialByte, sending, receiving, inbuff_index, msgtype);
    parse1(serialAvail, serialByte, sending, msgtype);

    /*
    static uint8_t _inbuff[128];

    if (receiving) {
        _inbuff[inbuff_index] = serialByte;
    }*/

    if (sending) {
        handleSerialInput(msgtype, state_phi, state_theta, state_psi);
    }

    updateSerialOutput();

    /*
    uint8_t motor_index = msgtype == 215 ? _inbuff[0] : 0;
    uint8_t motor_percent = msgtype == 215 ? _inbuff[1] : 0;

    float m1_val = armed ? m1_flying : motor_index == 1 ? motor_percent/100. : 0;
    float m2_val = armed ? m2_flying : motor_index == 2 ? motor_percent/100. : 0;
    float m3_val = armed ? m3_flying : motor_index == 3 ? motor_percent/100. : 0;
    float m4_val = armed ? m4_flying : motor_index == 4 ? motor_percent/100. : 0;

    stream_writeBrushedMotors(m1_pin, m2_pin, m3_pin, m4_pin, m1_val, m2_val, m3_val, m4_val);
    */
}
