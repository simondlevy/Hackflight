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

#include <Wire.h>
#include "copilot.h"
#include "copilot_arduino.h"

#include "cppsrc/Hackflight.hpp"
#include "cppsrc/receiver.hpp"
#include "cppsrc/serial.hpp"
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


// Hackflight object ===========================================================

static hf::Hackflight h(&receiver, &mixer, LED_PIN);

// Setup =======================================================================

void setup(void)
{
    Serial.begin(115200);
    Wire.begin();
    delay(100);

    copilot_startLed(LED_PIN);
    copilot_startReceiver();
    copilot_startImu();
    
    copilot_startBrushedMotors(MOTOR1_PIN,  MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN);

    h.addSensor(&imu);
    //h.addPidController(&levelPid);
    //h.addPidController(&ratePid);
    //h.addPidController(&yawPid);
    h.begin();
}

// Loop ======================================================================

void loop(void)
{

    copilot_micros = micros();

    copilot_updateReceiver();
    copilot_updateImu();

    float motors[4] = {};
    bool led = false;
    //static hf::serial_t serial;
    bool motorsReady = false;

    h.update(motors, led, /*serial,*/ motorsReady);

/*
    if (serial.input_ready) {
        copilot_handleSerialJnput(
          serial.input.w00,
          serial.input.w01,
          serial.input.w02,
          serial.input.w03);
    }

    if (serial.count > 0) {
        copilot_sendSerialOutput(
                serial.type,
                serial.count,
                serial.output.v00, serial.output.v01, serial.output.v02, serial.output.v03,
                serial.output.v04, serial.output.v05, serial.output.v06, serial.output.v07,
                serial.output.v08, serial.output.v09, serial.output.v10, serial.output.v11);
    }
*/

    copilot_setLed(led);

    copilot_writeBrushedMotors(motors[0], motors[1], motors[2], motors[3]);
}
