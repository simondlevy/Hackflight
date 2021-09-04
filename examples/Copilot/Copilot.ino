/*
   Hackflight using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "copilot.h"
#include "Debugger.hpp"

// LED ------------------------------------------------------------------------

static uint8_t LED_PIN = 13; // Teensy 4.0

// IMU ------------------------------------------------------------------------

#include <Wire.h>
#include <USFS_Master.h>

static USFS_Master usfs;

static void startImu(void)
{
    delay(100);
    if (!usfs.begin()) {
        while (true) {
            Debugger::printf("%s\n", usfs.getErrorString());
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

void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

static void updateReceiver(void)
{
    if (rx.timedOut(micros())) {
        copilot_receiverLostSignal = true;
    }

    else if (rx.gotNewFrame()) {

        float values[8] = {};

        rx.getChannelValues(values);

        copilot_receiverThrottle = values[0];
        copilot_receiverRoll = values[1];
        copilot_receiverPitch = values[2];
        copilot_receiverYaw = values[3];
    }
}


// Clock ---------------------------------------------------------------------

static uint32_t start_time_usec;

static void startClock(void)
{
    start_time_usec = micros();
}

static void updateClock(void)
{
    copilot_time_msec = millis();
    //copilot_time_sec = (micros() - start_time_usec) / 1e6;
}

// Called by Copilot ----------------------------------------------------------

void copilot_runMotors(float m1, float m2, float m3, float m4)
{
    // Debugger::printf("m1: %3.3f   m2: %3.3f   m3: %3.3f   m4: %3.3f\n", m1, m2, m3, m4);
}

void copilot_setLed(bool on)
{
    digitalWrite(LED_PIN, on);
}

void copilot_debug(float value)
{
    // Debugger::printf("%f\n", value);
}

static void powerPin(uint8_t id, uint8_t value)
{
    pinMode(id, OUTPUT);
    digitalWrite(id, value);
}


// Setup ------------------------------------------------------------------------

void setup(void)
{
    // Temporary Hack
    powerPin(21, HIGH);
    powerPin(22, LOW);

    // I^2C
    Wire.begin();

    // LED
    pinMode(LED_PIN, OUTPUT);

    // Serial comms
    Serial.begin(115200);

    // DSMX receiver
    Serial1.begin(115200);

    // IMU
    startImu(); 

    // Timing
    startClock();

    delay(1000);
}

// Loop --------------------------------------------------------------------------

void loop(void)
{
    updateClock();

    updateReceiver();

    updateImu();

    // Run Copilot code
    step();
}
