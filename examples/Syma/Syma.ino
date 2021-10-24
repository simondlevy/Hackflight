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

#include "newparser.hpp"
#include "debugger.hpp"
#include "stream_serial.h"
#include "copilot.h"

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
        float m4_flying)
{
    bool data_available = false;
    uint8_t data_byte = 0;
    uint8_t gcs_motor_index = 0;
    uint8_t gcs_motor_percent = 0;

    parse(state_phi,
          state_theta,
          state_psi,
          data_available,
          data_byte,
          gcs_motor_index,
          gcs_motor_percent);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (data_available) {
        stream_serialWrite(data_byte);
    }

    float m1_val = armed ? m1_flying : gcs_motor_index == 1 ? gcs_motor_percent/100. : 0;
    float m2_val = armed ? m2_flying : gcs_motor_index == 2 ? gcs_motor_percent/100. : 0;
    float m3_val = armed ? m3_flying : gcs_motor_index == 3 ? gcs_motor_percent/100. : 0;
    float m4_val = armed ? m4_flying : gcs_motor_index == 4 ? gcs_motor_percent/100. : 0;

    stream_writeBrushedMotors(m1_pin, m2_pin, m3_pin, m4_pin, m1_val, m2_val, m3_val, m4_val);
}
