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

#include "copilot_extra.h"

#include "cppsrc/Hackflight.hpp"
#include "cppsrc/receiver.hpp"
#include "cppsrc/mixers/quadxmw.hpp"
#include "cppsrc/pidcontrollers/rate.hpp"
#include "cppsrc/pidcontrollers/yaw.hpp"
#include "cppsrc/pidcontrollers/level.hpp"
#include "cppsrc/sensors/usfs.hpp"

// LED =======================================================================

static uint8_t LED_PIN = A4;

// Motors =======================================================================

static uint8_t MOTOR1_PIN = 13;
static uint8_t MOTOR2_PIN = 16;
static uint8_t MOTOR3_PIN = 3;
static uint8_t MOTOR4_PIN = 11;

// Receiver ===================================================================

static constexpr float DEMAND_SCALE = 4.0f;
static constexpr float SOFTWARE_TRIM[3] = {0, 0.05, 0.035};

static hf::Receiver receiver = hf::Receiver(DEMAND_SCALE, SOFTWARE_TRIM);

// Mixer =======================================================================

static hf::MixerQuadXMW mixer;

// PID controllers =============================================================

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

// Sensors =====================================================================

static hf::USFS imu;

// Serial tasks ================================================================

hf::SerialTask gcsTask;

// Hackflight object ===========================================================

static hf::Hackflight h(&receiver, &mixer, LED_PIN);

// Setup =======================================================================

void setup(void)
{
    copilot_startWire();
    copilot_startUsfs(); 
    copilot_startDsmrx();
    copilot_startLed(LED_PIN);
    copilot_startSerial();

    copilot_startBrushedMotor(MOTOR1_PIN);
    copilot_startBrushedMotor(MOTOR2_PIN);
    copilot_startBrushedMotor(MOTOR3_PIN);
    copilot_startBrushedMotor(MOTOR4_PIN);

    h.addSensor(&imu);
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);
    h.addSerialTask(&gcsTask);
    h.begin();
}

// Loop ======================================================================

void loop(void)
{
    copilot_updateDsmrx();
    copilot_updateUsfs();
    copilot_updateSerial();
    copilot_updateClock();

    float motors[4] = {};
    bool led = false;
    serial_t serial = {};

    h.update(motors, led, serial);

    copilot_handleSerial(serial);

    copilot_setLed(LED_PIN, led);

    copilot_writeBrushedMotor(MOTOR1_PIN, motors[0]);
    copilot_writeBrushedMotor(MOTOR2_PIN, motors[1]);
    copilot_writeBrushedMotor(MOTOR3_PIN, motors[2]);
    copilot_writeBrushedMotor(MOTOR4_PIN, motors[3]);
}
