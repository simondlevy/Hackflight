/*
 * SBUS output
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <sbus.h>

static const uint8_t  SBUS_CHANNELS = 16;
static const uint16_t SBUS_MIN      = 172;
static const uint16_t SBUS_MAX      = 1811;

SbusTx sbus_out(&Serial1, SBUS_RX_PIN, SBUS_TX_PIN);

static uint16_t scale(float txval)
{
    return ((txval + 1) / 2) * (SBUS_MAX - SBUS_MIN) + SBUS_MIN;
}

void stream_startSbusOut(void) {

    sbus_out.Begin();
}

void stream_writeSbus(float thr, float rol, float pit, float yaw, float aux) {

    std::array<uint16_t, SBUS_CHANNELS> sbusvals;

    sbusvals[0] = scale(thr);
    sbusvals[1] = scale(rol);
    sbusvals[2] = scale(pit);
    sbusvals[3] = scale(yaw);
    sbusvals[4] = scale(aux);

    for (uint8_t k=5; k<SBUS_CHANNELS; ++k) {
        sbusvals[k] = SBUS_MIN;
    }

    sbus_out.tx_channels(sbusvals);
    sbus_out.Write();
}

