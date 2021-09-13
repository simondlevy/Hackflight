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

#include "copilot.h"

#include "cppsrc/Hackflight.hpp"
#include "cppsrc/boards/realboards/arduino_serial/arduino/ladybugfc.hpp"
#include "cppsrc/receivers/arduino/dsmx/dsmx_serial1.hpp"
#include "cppsrc/mixers/quad/xmw.hpp"
#include "cppsrc/pidcontrollers/rate.hpp"
#include "cppsrc/pidcontrollers/yaw.hpp"
#include "cppsrc/pidcontrollers/level.hpp"
#include "cppsrc/sensors/usfs.hpp"
#include "cppsrc/motors/arduino/brushed.hpp"

// LED ------------------------------------------------------------------------

static uint8_t LED_PIN = A4;

// Receiver ===================================================================

static constexpr uint8_t CHANNEL_MAP[6] = {0, 1, 2, 3, 6, 4};
static constexpr float DEMAND_SCALE = 4.0f;
static constexpr float SOFTWARE_TRIM[3] = {0, 0.05, 0.035};

static hf::DSMX_Receiver_Serial1 receiver = 
    hf::DSMX_Receiver_Serial1(CHANNEL_MAP, DEMAND_SCALE, SOFTWARE_TRIM);  

// Board =======================================================================

// Bluetooth comms over Serial2
static hf::LadybugFC board = hf::LadybugFC(&Serial2);  

// Motors  =====================================================================

static hf::ArduinoBrushedMotor motor1 = hf::ArduinoBrushedMotor(13);
static hf::ArduinoBrushedMotor motor2 = hf::ArduinoBrushedMotor(A2);
static hf::ArduinoBrushedMotor motor3 = hf::ArduinoBrushedMotor(3);
static hf::ArduinoBrushedMotor motor4 = hf::ArduinoBrushedMotor(11);

// Mixer =======================================================================

static hf::MixerQuadXMW mixer = hf::MixerQuadXMW(&motor1, &motor2, &motor3, &motor4);

// PID controllers =============================================================

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

// Sensors =====================================================================

static hf::USFS imu;

// Serial tasks ================================================================

hf::SerialTask gcsTask;
hf::SerialTask telemetryTask = hf::SerialTask(true);

// Hackflight object ===========================================================

static hf::Hackflight h(&board, &receiver, &mixer);

// Copilot support =============================================================

void copilot_setLed(bool on)
{
    digitalWrite(LED_PIN, on);
}

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


// Setup =======================================================================

void setup(void)
{
    // I^2C
    Wire.begin();

    // IMU
    startImu(); 

    // LED
    pinMode(LED_PIN, OUTPUT);

    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    // Add serial tasks
    h.addSerialTask(&gcsTask);
    h.addSerialTask(&telemetryTask);

    // Start Hackflight firmware
    h.begin();
}

// Loop ======================================================================

void loop(void)
{
    updateImu();

    h.update();
}
