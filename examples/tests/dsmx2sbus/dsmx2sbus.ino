/*
   Test DSMX => SBUS translation

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <SBUS.h>
#include <DSMRX.h>

static const uint8_t CHANNELS_IN  = 8;
static const uint8_t CHANNELS_OUT = 16;

SBUS sbus = SBUS(Serial1);

DSM2048 rx;

void serialEvent2(void)
{
    while (Serial2.available()) {
        rx.handleSerialEvent(Serial2.read(), micros());
    }
}

void setup(void)
{
    sbus.begin();

    // For DSMX in
    Serial2.begin(115000);

    // For debugging
    Serial.begin(115000);
}

void loop(void)
{
    float invals[CHANNELS_IN] = {0};

    rx.getChannelValuesNormalized(invals, CHANNELS_IN);

    static float outvals[CHANNELS_OUT];

    outvals[0] = invals[0]; // Throttle
    outvals[1] = invals[1]; // Roll
    outvals[2] = invals[2]; // Pitch
    outvals[3] = invals[3]; // Yaw

    outvals[5] = invals[6]; // DSXM Aux1 => SBUS Aux2

    sbus.writeCal(outvals);

    delay(10);
}

