/*
 * SBUS output
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <sbus.h>

SbusTx sbus_out(&Serial2);

static uint16_t scale(float txval, uint16_t sbus_min, uint16_t sbus_max)
{
    return ((txval + 1) / 2) * (sbus_max - sbus_min) + sbus_min;
}

void stream_startSbusOut(uint8_t rxpin, uint8_t txpin) {

    sbus_out.Begin(rxpin, txpin);
}

void stream_writeSbus(
        uint16_t sbus_min,
        uint16_t sbus_max,
        float thr,
        float rol,
        float pit,
        float yaw,
        float aux) {

    std::array<uint16_t, 16> sbusvals;

    sbusvals[0] = scale(thr, sbus_min, sbus_max);
    sbusvals[1] = scale(rol, sbus_min, sbus_max);
    sbusvals[2] = scale(pit, sbus_min, sbus_max);
    sbusvals[3] = scale(yaw, sbus_min, sbus_max);
    sbusvals[4] = scale(aux, sbus_min, sbus_max);

    for (uint8_t k=5; k<16; ++k) {
        sbusvals[k] = sbus_min;
    }

    sbus_out.tx_channels(sbusvals);
    sbus_out.Write();
}

