/*
 * SBUS output
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <sbus.h>
#include "arduino_debugger.hpp"

SbusTx sbus_out(&Serial2);

#ifdef ESP32
void sbusOutStart(uint8_t rxpin, uint8_t txpin) {

    sbus_out.Begin(rxpin, txpin);
}
#else
void sbusOutStart(void) {

    sbus_out.Begin();
}
#endif

void sbusWrite(uint16_t c1, uint16_t c2, uint16_t c3, uint16_t c4, uint16_t c5, uint16_t c6)
{
    std::array<uint16_t, 16> sbusvals;

    //Debugger::printf("%04d %04d %04d %04d %04d %04d\n", c1, c2, c3, c4, c5, c6);

    sbusvals[0] = c1;
    sbusvals[1] = c2;
    sbusvals[2] = c3;
    sbusvals[3] = c4;
    sbusvals[4] = c5;
    sbusvals[5] = c6;

    sbus_out.tx_channels(sbusvals);
    sbus_out.Write();
}
