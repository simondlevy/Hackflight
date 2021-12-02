/*
 * SBUS output tester
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <sbus.h>

static const uint8_t  SBUS_CHANNELS = 16;
static const uint16_t SBUS_MIN = 172;
static const uint16_t SBUS_MAX = 1811;

SbusTx sbus_out(&Serial1
#ifdef ESP32
        , 4, 14
#endif
        );

void setup() {

    sbus_out.Begin();

    Serial.begin(115000);
}

void loop() {

    std::array<uint16_t, SBUS_CHANNELS> sbusvals;

    static uint16_t sbusval;
    static int8_t dir;

    if (sbusval == 0) {
        sbusval = SBUS_MIN;
        dir = +1;
    }

    sbusval += dir;

    if (sbusval == SBUS_MIN) {
        dir = +1;
    }

    if (sbusval == SBUS_MAX) {
        dir = -1;
    }

    Serial.println(sbusval);

    for (uint8_t k=0; k<SBUS_CHANNELS; ++k) {
        sbusvals[k] = SBUS_MIN;
    }

    sbusvals[0] = sbusval;

    sbus_out.tx_channels(sbusvals);

    sbus_out.Write();

    delay(5);
}

