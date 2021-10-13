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

#include "HF_full.hpp"
#include "hf_mixers/quad/xmw.hpp"
#include "hf_pidcontrollers/rate.hpp"
#include "hf_pidcontrollers/yaw.hpp"
#include "hf_pidcontrollers/level.hpp"
#include "hf_sensors/imu.hpp"

#include "stream_serial.h"
#include "stream_i2c.h"
#include "stream_motors.h"
#include "stream_receiver.h"
#include "stream_imu.h"
#include "stream_led.h"

static const uint8_t MOTOR_PINS[4] = {13, 16, 3, 11};

static uint32_t LED_PIN = 18;

static constexpr float SCALE = 4.0f;
static constexpr float TRIM[3] = {0, 0.05, 0.045};
static hf::Receiver receiver = hf::Receiver(SCALE, TRIM);

static hf::MixerQuadXMW mixer;

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

static hf::IMU imu;

static hf::HackflightFull h(&receiver, &mixer);

static bool running;

void setup(void)
{
    h.addSensor(&imu);
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);
}

void loop(void)
{
    if (!running) {

        stream_startSerial();
        stream_startI2C();
        stream_startReceiver();
        stream_startImu();
        stream_startBrushedMotors(MOTOR_PINS);
        stream_startLed(LED_PIN);

        running = true;
    }

    stream_updateImu();
    stream_updateReceiver();

    bool ledval = false;
    bool serialTaskReady = false;
    hf::motors_t motors = {};

    h.update(micros(), motors, ledval);
    h.serialParse(motors);

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (h.serialAvailable() > 0) {
        stream_serialWrite(h.serialRead());
    }

    if (motors.ready) {
        stream_writeBrushedMotors(MOTOR_PINS, motors.values);
    }

    stream_writeLed(LED_PIN, ledval);
}
