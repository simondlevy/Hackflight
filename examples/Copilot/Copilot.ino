/*
   Hackflight using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "copilot.h"

#include <DSMRX.h>

static DSM2048 rx;

void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

// Used by Copilot ---------------------------------

float copilot_time = 0;

float copilot_receiverThrottle = 0;
float copilot_receiverRoll = 0;
float copilot_receiverPitch = 0;
float copilot_receiverYaw = 0;

bool copilot_receiverLostSignal = false;
bool copilot_receiverReady = true;
bool copilot_receiverInArmedState = true;
bool copilot_receiverInactive = false;

float copilot_gyrometerX = 0;
float copilot_gyrometerY = 0;
float copilot_gyrometerZ = 0;

float copilot_quaternionW = 0;
float copilot_quaternionX = 0;
float copilot_quaternionY = 0;
float copilot_quaternionZ = 0;

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
    reportMotor(m1, 1);
    reportMotor(m2, 2);
    reportMotor(m3, 3);
    reportMotor(m4, 4);
  
    Serial.println();
}

static void runReceiver(void)
{
    static const uint8_t CHANNELS = 8;

    if (rx.timedOut(micros())) {
        Serial.println("*** TIMED OUT ***");
    }

    else if (rx.gotNewFrame()) {

        float values[CHANNELS];

        rx.getChannelValuesNormalized(values, CHANNELS);

        copilot_receiverThrottle = values[0];
        copilot_receiverRoll = values[1];
        copilot_receiverPitch = values[2];
        copilot_receiverYaw = values[3];
    }
}

void setup(void)
{
    Serial.begin(115200);

    Serial1.begin(115200);
}

void loop(void)
{
    runReceiver();

    // Run Copilot code
    step();
}
