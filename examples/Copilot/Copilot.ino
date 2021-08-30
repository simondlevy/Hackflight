/*
   Hackflight using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "copilot.h"

// IMU ---------------------------------------------

#include <Wire.h>
#include <USFS_Master.h>

static const uint8_t  MAG_RATE       = 100;  // Hz
static const uint16_t ACCEL_RATE     = 200;  // Hz
static const uint16_t GYRO_RATE      = 200;  // Hz
static const uint8_t  BARO_RATE      = 50;   // Hz
static const uint8_t  Q_RATE_DIVISOR = 3;    // 1/3 gyro rate
 
USFS_Master usfs = USFS_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR);

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
        Serial.println("*** RECEIVER TIMED OUT ***");
    }

    else if (rx.gotNewFrame()) {

        float values[CHANNELS];

        rx.getChannelValuesNormalized(values, CHANNELS);

        copilot_receiverChannel1 = values[0];
        copilot_receiverChannel2 = values[1];
        copilot_receiverChannel3 = values[2];
        copilot_receiverChannel4 = values[3];
    }
}

static void startImu(void)
{
    Wire.begin();
    delay(100);
    if (!usfs.begin()) {
        while (true) {
            Serial.println(usfs.getErrorString());
        }
    }
}

void setup(void)
{
    // Serial comms
    Serial.begin(115200);

    // DSMX receiver
    Serial1.begin(115200);

    startImu(); 
}

void loop(void)
{
    runReceiver();

    // Run Copilot code
    step();
}
