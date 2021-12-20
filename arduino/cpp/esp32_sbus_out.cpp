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

void stream_sbusOutStart(uint8_t rxpin, uint8_t txpin) {

    Serial.begin(115200);
    //sbus_out.Begin(rxpin, txpin);
}

void stream_sbusWrite(
        float ch01,
        float ch02,
        float ch03,
        float ch04,
        float ch05,
        float ch06,
        float ch07,
        float ch08,
        float ch09,
        float ch10,
        float ch11,
        float ch12,
        float ch13,
        float ch14,
        float ch15,
        float ch16)
{
    std::array<uint16_t, 16> sbusvals;

    sbusvals[0] = (uint16_t)ch01;
    sbusvals[1] = (uint16_t)ch02;
    sbusvals[2] = (uint16_t)ch03;
    sbusvals[3] = (uint16_t)ch04;
    sbusvals[4] = (uint16_t)ch05;
    sbusvals[5] = (uint16_t)ch06;
    sbusvals[6] = (uint16_t)ch07;
    sbusvals[7] = (uint16_t)ch08;
    sbusvals[8] = (uint16_t)ch09;
    sbusvals[9] = (uint16_t)ch10;
    sbusvals[10] = (uint16_t)ch11;
    sbusvals[11] = (uint16_t)ch12;
    sbusvals[12] = (uint16_t)ch13;
    sbusvals[13] = (uint16_t)ch14;
    sbusvals[14] = (uint16_t)ch15;
    sbusvals[15] = (uint16_t)ch16;

    //sbus_out.tx_channels(sbusvals);
    //sbus_out.Write();

    Debugger::printf("%04d\n", sbusvals[0]);
}

void stream_ignore(bool timedOut)
{
}
