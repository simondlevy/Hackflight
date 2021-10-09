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
#include "hf_motors/arduino/brushed.hpp"
#include "hf_sensors/usfs.hpp"

#include <Wire.h>
#include <DSMRX.h>
#include <USFS_Master.h>

// LED ========================================================================

static uint32_t LED_PIN = 18;

// Receiver ===================================================================

static constexpr float DEMAND_SCALE = 4.0f;
static constexpr float SOFTWARE_TRIM[3] = {0, 0.05, 0.035};

bool copilot_receiverGotNewFrame;
bool copilot_receiverLostSignal;
float copilot_receiverThrottle;
float copilot_receiverRoll;
float copilot_receiverPitch;
float copilot_receiverYaw;
float copilot_receiverAux1;
float copilot_receiverAux2;

static DSM2048 rx;

static bool rxReady;

static hf::Receiver receiver = hf::Receiver(DEMAND_SCALE, SOFTWARE_TRIM);  

void serialEvent1(void)
{
    rxReady = true;

    while (Serial1.available()) {

        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

static void updateReceiver(void)
{
    if (rx.timedOut(micros())) {
        if (rxReady) {
            copilot_receiverLostSignal = true;
        }
    }

    else if (rx.gotNewFrame()) {

        float rawvals[8];

        rx.getChannelValues(rawvals, 8);

        copilot_receiverThrottle = rawvals[0];
        copilot_receiverRoll     = rawvals[1];
        copilot_receiverPitch    = rawvals[2];
        copilot_receiverYaw      = rawvals[3];
        copilot_receiverAux1     = rawvals[6];
        copilot_receiverAux2     = rawvals[4];
    }
}

static void startReceiver(void)
{
    Serial1.begin(115200);
    copilot_receiverLostSignal = false;
}

// Motors  =====================================================================

static hf::ArduinoBrushedMotor motor1 = hf::ArduinoBrushedMotor(13);
static hf::ArduinoBrushedMotor motor2 = hf::ArduinoBrushedMotor(A2);
static hf::ArduinoBrushedMotor motor3 = hf::ArduinoBrushedMotor(3);
static hf::ArduinoBrushedMotor motor4 = hf::ArduinoBrushedMotor(11);

static void startMotors(void)
{
    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
}

static void runMotors(float * mvals)
{
    motor1.write(mvals[0]);
    motor2.write(mvals[1]);
    motor3.write(mvals[2]);
    motor4.write(mvals[3]);
}

// Mixer =======================================================================

static hf::MixerQuadXMW mixer;

// PID controllers =============================================================

static hf::RatePid ratePid = hf::RatePid(0.225, 0.001875, 0.375);
static hf::YawPid yawPid = hf::YawPid(1.0625, 0.005625f);
static hf::LevelPid levelPid = hf::LevelPid(0.20f);

// IMU =========================================================================

static hf::USFS imu;

static USFS_Master usfs;

bool copilot_usfsGotGyrometer;
bool copilot_usfsGotQuaternion;
float copilot_usfsGyrometerX;
float copilot_usfsGyrometerY;
float copilot_usfsGyrometerZ;
float copilot_usfsQuaternionW;
float copilot_usfsQuaternionX;
float copilot_usfsQuaternionY;
float copilot_usfsQuaternionZ;

static void startImu(void)
{
    // Start the USFS in master mode, no interrupt
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
            delay(100);
        }
    }
}

static void updateImu(void)
{
    usfs.checkEventStatus();

    if (usfs.gotError()) {
        while (true) {
            Serial.print("ERROR: ");
            Serial.println(usfs.getErrorString());
        }
    }

    copilot_usfsGotGyrometer = usfs.gotGyrometer();

    if (copilot_usfsGotGyrometer) {
        // Returns degrees / sec
        usfs.readGyrometer(
             copilot_usfsGyrometerX,
             copilot_usfsGyrometerY,
             copilot_usfsGyrometerZ);
    }

    copilot_usfsGotQuaternion = usfs.gotQuaternion();

    if (copilot_usfsGotQuaternion) {
        usfs.readQuaternion(
             copilot_usfsQuaternionW,
             copilot_usfsQuaternionX,
             copilot_usfsQuaternionY,
             copilot_usfsQuaternionZ);
    }
}

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

static void startSerial(void)
{
    Serial.begin(115200);
}

// LED support =================================================================

static void startLed(void)
{
    pinMode(LED_PIN, OUTPUT);    
}

// I^2C support ===============================================================

static void startI2C(void)
{
    Wire.begin();
    delay(100);
}

// Setup =======================================================================

void setup(void)
{
    startSerial();
    startLed();
    startI2C();
    startReceiver();
    startMotors();
    startImu();

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
    updateImu();
    updateReceiver();

    bool led = false;
    static float mvals[4];

    h.update(micros(), mvals, &led);

    //printf("%3.3f  %3.3f  %3.3f  %3.3f\n", mvals[0], mvals[1], mvals[2], mvals[3]);

    runMotors(mvals);

    digitalWrite(LED_PIN, led);
}
