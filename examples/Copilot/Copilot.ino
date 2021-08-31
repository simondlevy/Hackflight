/*
   Hackflight using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "copilot.h"

// IMU ---------------------------------------------

#include <Wire.h>
#include <USFS_Master.h>

static USFS_Master usfs;

// Receiver ----------------------------------------

#include <DSMRX.h>

static DSM2048 rx;

void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

static void reportMotor(float value, uint8_t index)
{
    Serial.print("m");
    Serial.print(index);
    Serial.print(": ");
    Serial.print(value);
    Serial.print("    ");
}

// Called by Copilot
void copilot_runMotors(float m1, float m2, float m3, float m4)
{
    return;
    reportMotor(m1, 1);
    reportMotor(m2, 2);
    reportMotor(m3, 3);
    reportMotor(m4, 4);
  
    Serial.println();
}

void copilot_debug(float phi)
{
    printf("Phi: %+3.3f\n", phi);
}

static void runReceiver(void)
{
    static const uint8_t CHANNELS = 8;

    if (rx.timedOut(micros())) {
        // copilot_receiverLostSignal = true;
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

static void startImu(void)
{
    delay(100);
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
        }
    }
}

static void runImu(void)
{
    usfs.checkEventStatus();

    if (usfs.gotGyrometer()) {
        usfs.readGyrometer(copilot_gyrometerX, copilot_gyrometerY, copilot_gyrometerZ);
    }

    if (usfs.gotQuaternion()) {
        usfs.readQuaternion(copilot_quaternionW, copilot_quaternionX, copilot_quaternionY, copilot_quaternionZ);
    }
}

void setup(void)
{
    // Start I^2C
    Wire.begin();

    // Serial comms
    Serial.begin(115200);

    // DSMX receiver
    Serial1.begin(115200);

    // IMU
    startImu(); 
}

void loop(void)
{
    runReceiver();

    runImu();

    // Run Copilot code
    step();
}
