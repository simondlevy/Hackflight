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
#include "hf_receiver.hpp"
#include "hf_mixers/quad/xmw.hpp"
#include "hf_pidcontrollers/rate.hpp"
#include "hf_pidcontrollers/yaw.hpp"
#include "hf_pidcontrollers/level.hpp"
#include "hf_sensors/imu.hpp"

#include "stream_motors.h"
#include "stream_receiver.h"
#include "stream_imu.h"
#include "stream_led.h"

#include <Wire.h>
#include <USFS_Master.h>

// Motors ========================================================================

static const uint8_t MOTOR_PINS[4] = {13, 16, 3, 11};

// LED ========================================================================

static uint32_t LED_PIN = 18;

// Receiver ===================================================================

static constexpr float SCALE = 4.0f;
static constexpr float TRIM[3] = {0, 0.05, 0.035};

static hf::Receiver receiver = hf::Receiver(SCALE, TRIM);

// Mixer =======================================================================

static hf::MixerQuadXMW mixer;

// PID controllers =============================================================

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

// IMU =========================================================================

static hf::IMU imu;

// Serial tasks ================================================================

hf::SerialTask gcsTask;

// Hackflight object ===========================================================

static hf::HackflightFull h(&receiver, &mixer);

// Serial comms support ========================================================

namespace hf {

    bool serialAvailable(void)
    {
        return Serial.available();
    }

    uint8_t serialRead(void)
    {
        return Serial.read();
    }

    void serialWrite(uint8_t b)
    {
        Serial.write(b);
    }

} // namespace hf

// Setup =======================================================================

void setup(void)
{
    Serial.begin(115200);

    Wire.begin();
    delay(100);

    stream_startReceiver();
    stream_startImu();
    stream_startBrushedMotors(MOTOR_PINS);
    stream_startLed(LED_PIN);

    // Add sensors
    h.addSensor(&imu);

    // Add PID controllers
    h.addPidController(&levelPid);
    h.addPidController(&ratePid);
    h.addPidController(&yawPid);

    // Add serial tasks
    h.addSerialTask(&gcsTask);

    // Start Hackflight firmware
    h.begin();
}

// Loop ======================================================================

void loop(void)
{
    stream_updateImu();
    stream_updateReceiver();

    bool ledval = false;
    static float motorvals[4];

    h.update(micros(), motorvals, &ledval);

    stream_writeBrushedMotors(MOTOR_PINS, motorvals);

    stream_writeLed(LED_PIN, ledval);
}
