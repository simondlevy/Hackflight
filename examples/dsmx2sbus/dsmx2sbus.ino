/*
   Test DSMX => SBUS translation

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include <DSMRX.h>
#include <SBUS.h>

DSM2048 rx;

SBUS sbus = SBUS(Serial2);

void serialEvent1(void)
{
    while (Serial1.available()) {
        rx.handleSerialEvent(Serial1.read(), micros());
    }
}

void setup(void)
{
    Serial1.begin(115000);

    Serial.begin(115000);

    sbus.begin();
}

void loop(void)
{
    static float outvals[16] = {};

    if (rx.timedOut(micros())) {
        Serial.println("*** TIMED OUT ***");
    }

    else if (rx.gotNewFrame()) {

        float invals[8] = {};

        rx.getChannelValuesNormalized(invals, 8);

        outvals[0] = invals[0];
        outvals[1] = invals[1];
        outvals[2] = invals[2];
        outvals[3] = invals[3];
        outvals[4] = invals[6];
        outvals[5] = invals[4];
    }

    Serial.print(outvals[0]);
    Serial.print(" " );
    Serial.print(outvals[1]);
    Serial.print(" " );
    Serial.print(outvals[2]);
    Serial.print(" " );
    Serial.print(outvals[3]);
    Serial.print(" " );
    Serial.print(outvals[4]);
    Serial.print(" " );
    Serial.print(outvals[5]);
    Serial.println();

    sbus.writeCal(outvals);
}

