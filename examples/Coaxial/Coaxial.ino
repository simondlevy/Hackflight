/*
   Hackflight sketch for experimental coaxial copter with Teensy 4.0 and
   Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/SpektrumDSM 

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include "hackflight.hpp"
#include "receivers/arduino/dsmx/dsmx_serial1.hpp"

#include <rft_boards/realboards/arduino/teensy.hpp>
#include <rft_closedloops/passthru.hpp>

#include "actuators/mixers/quads/coaxial.hpp"
#include "motors/brushed.hpp"
#include "motors/servo.hpp"

static const uint8_t SERVO1_PIN = 22;
static const uint8_t SERVO2_PIN = 23;
static const uint8_t MOTOR1_PIN = 8;
static const uint8_t MOTOR2_PIN = 9;

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;

static hf::ServoMotor servo1 = hf::ServoMotor(SERVO1_PIN);
static hf::ServoMotor servo2 = hf::ServoMotor(SERVO2_PIN);

static hf::BrushedMotor motor1 = hf::BrushedMotor(MOTOR1_PIN);
static hf::BrushedMotor motor2 = hf::BrushedMotor(MOTOR2_PIN);

static rft::Teensy40 board;

static hf::DSMX_Receiver_Serial1 receiver = hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE);  

static hf::CoaxialMixer mixer = hf::CoaxialMixer(&servo1, &servo2, &motor1, &motor2);

static hf::Hackflight h(&board, &receiver, &mixer);

static rft::PassthruController passthru;

void setup(void)
{
    h.addClosedLoopController(&passthru);

    h.begin();
}

void loop(void)
{
    h.update();
}
