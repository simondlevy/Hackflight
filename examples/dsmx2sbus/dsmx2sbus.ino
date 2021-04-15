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

    sbus.begin();
}

void loop(void)
{
    static float outvals[16] = {};

    if (rx.timedOut(micros())) {
        Serial.println("*** TIMED OUT ***");
    }

    else if (rx.gotNewFrame()) {

        float values[8];

        rx.getChannelValuesNormalized(values, 8);

        for (int k=0; k<4; ++k) {
            outvals[k] = values[k];
            Serial.print("Ch. ");
            Serial.print(k+1);
            Serial.print(": ");
            Serial.print(values[k]);
            Serial.print("    ");
        }

        Serial.println();

    }

    sbus.writeCal(outvals);
}

