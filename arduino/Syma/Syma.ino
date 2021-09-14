/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/EM7180
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

#include "cppsrc/motors/arduino/brushed.hpp"

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

// IMU ------------------------------------------------------------------------

#include <Wire.h>
#include <USFS_Master.h>

static USFS_Master usfs;

static void startImu(void)
{
    delay(100);
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
            delay(500);
        }
    }
}

static void updateImu(void)
{
    usfs.checkEventStatus();

    if (usfs.gotGyrometer()) {
        usfs.readGyrometer(
                copilot_gyrometerX,
                copilot_gyrometerY,
                copilot_gyrometerZ);
    }

    if (usfs.gotQuaternion()) {
        usfs.readQuaternion(
                copilot_quaternionW,
                copilot_quaternionX,
                copilot_quaternionY,
                copilot_quaternionZ);
    }
}

// Receiver -------------------------------------------------------------------

#include <DSMRX.h>

static DSM2048 rx;
static uint32_t timeouts;

void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

static void startReceiver(void)
{
    Serial1.begin(115200);

    copilot_receiverLostSignal = false;
}

static void updateReceiver(void)
{
    if (rx.timedOut(micros())) {
        timeouts++;
        if (timeouts > 10) {
            copilot_receiverLostSignal = true;
        }
    }

    else if (rx.gotNewFrame()) {

        float values[8] = {};

        rx.getChannelValues(values);

        copilot_receiverThrottle = values[0];
        copilot_receiverRoll = values[1];
        copilot_receiverPitch = values[2];
        copilot_receiverYaw = values[3];
        copilot_receiverAux1 = values[6];
    }
}

// Serial comms ----------------------------------------------------------------

static void startSerial(void)
{
    Serial.begin(115200);
}

void updateSerial(void)
{
    copilot_serialAvailable = Serial.available();

    if (copilot_serialAvailable) {
        copilot_serialByte = Serial.read();
    }
}

void copilot_serialWrite(uint8_t b)
{
    Serial.write(b);
}

// LED ------------------------------------------------------------------------

void copilot_setLed(bool on)
{
    digitalWrite(LED_PIN, on);
}

// clock  ----------------------------------------------------------------------

static void updateClock(void)
{
    copilot_time_sec = micros() / 1.e6f;
}

// Setup =======================================================================

void setup(void)
{
    Wire.begin();

    startImu(); 

    startReceiver();

    startMotors();

    pinMode(LED_PIN, OUTPUT);

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
