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
        uint8_t parserbyte,
        uint8_t msgtype,
        bool sending,
        uint8_t payindex)
{
    static uint8_t motor_index;
    static uint8_t motor_percent;

    if (sending) {
        handleSerialInput(msgtype, state_phi, state_theta, state_psi);
    }

    else if (msgtype == 215) {

        if (payindex == 1) {
            motor_index = parserbyte;
        }

        if (payindex == 2) {
            motor_percent = parserbyte;
        }
    }

    updateSerialOutput();

    float m1_val = armed ? m1_flying : motor_index == 1 ? motor_percent/100. : 0;
    float m2_val = armed ? m2_flying : motor_index == 2 ? motor_percent/100. : 0;
    float m3_val = armed ? m3_flying : motor_index == 3 ? motor_percent/100. : 0;
    float m4_val = armed ? m4_flying : motor_index == 4 ? motor_percent/100. : 0;

    stream_writeBrushedMotors(m1_pin, m2_pin, m3_pin, m4_pin, m1_val, m2_val, m3_val, m4_val);
}
