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
#include "copilot_arduino.h"

#include "cppsrc/Hackflight.hpp"
#include "cppsrc/receiver.hpp"
#include "cppsrc/mixers/quadxmw.hpp"
#include "cppsrc/pidcontrollers/rate.hpp"
#include "cppsrc/pidcontrollers/yaw.hpp"
#include "cppsrc/pidcontrollers/level.hpp"
#include "cppsrc/sensors/usfs.hpp"
#include "cppsrc/motors/arduino/brushed.hpp"

#include <Wire.h>

// LED =======================================================================

static uint8_t LED_PIN = A4;

// Receiver ===================================================================

static constexpr float DEMAND_SCALE = 4.0f;
static constexpr float SOFTWARE_TRIM[3] = {0, 0.05, 0.035};

static hf::Receiver receiver = hf::Receiver(DEMAND_SCALE, SOFTWARE_TRIM);

// Motors  =====================================================================

static hf::ArduinoBrushedMotor motors[4] = { hf::ArduinoBrushedMotor(13)
                                           , hf::ArduinoBrushedMotor(A2)
                                           , hf::ArduinoBrushedMotor(3)
                                           , hf::ArduinoBrushedMotor(11)
                                           };

static void startMotors(void)
{
    motors[0].begin();
    motors[1].begin();
    motors[2].begin();
    motors[3].begin();
}

void copilot_writeMotor(uint8_t index, float value)
{
    motors[index].write(value);
}

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

static hf::Hackflight h(&receiver, &mixer);

// Setup =======================================================================

void setup(void)
{
    Wire.begin();

    startImu(); 

    startReceiver();

    startMotors();

    startLed(LED_PIN);

    h.addSensor(&imu);

    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    h.addSerialTask(&gcsTask);

    startSerial();

    h.begin();
}

// Loop ======================================================================

void loop(void)
{
    updateReceiver();

    updateImu();

    updateSerial();

    updateClock();

    h.update();
}
