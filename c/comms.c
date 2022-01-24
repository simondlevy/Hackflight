/*
   Platform-independent support for serial comms that can't be done in Haskell

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <stdint.h>
#include <string.h>

#include "arduino_debugger.hpp"

static uint8_t buffer[256];
static uint8_t bufidx;

void commsWrite(uint8_t *, uint8_t);

static void putbuf(uint8_t byte)
{
    buffer[bufidx++] = byte;
}

static void serializeByte(uint8_t & crc, uint8_t byte)
{
    putbuf(byte);
    crc ^= byte;
}

static void serializeFloat(uint8_t & crc, float floatval)
{
    int16_t intval = (int16_t)floatval;

    serializeByte(crc, intval     & 0xFF);
    serializeByte(crc, intval>>8  & 0xFF);
}

void commsSendAttitude(float angx , float angy , float heading)
{
    bufidx = 0;

    putbuf('$');
    putbuf('M');
    putbuf('>');
    putbuf(6);
    putbuf(108);

    uint8_t crc = 6 ^ 108;

    serializeFloat(crc, angx);
    serializeFloat(crc, angy);
    serializeFloat(crc, heading);

    serializeByte(crc, crc);

    commsWrite(buffer, bufidx);
}


void commsSend(
          uint8_t direction
        , uint8_t size  
        , uint8_t type  
        , float v1
        , float v2
        , float v3
        , float v4
        , float v5
        , float v6
        )
{
    bufidx = 0;

    putbuf('$');
    putbuf('M');
    putbuf(direction);
    putbuf(size);
    putbuf(type);

    uint8_t crc = size ^ type;

    uint8_t nvals = size / 2;

    if (nvals > 0) serializeFloat(crc, v1);
    if (nvals > 1) serializeFloat(crc, v2);
    if (nvals > 2) serializeFloat(crc, v3);
    if (nvals > 3) serializeFloat(crc, v4);
    if (nvals > 4) serializeFloat(crc, v5);
    if (nvals > 5) serializeFloat(crc, v6);

    Debugger::printf("%f %f %f %f %f %f\n", v1, v2, v3, v4, v5, v6);

    serializeByte(crc, crc);

    commsWrite(buffer, bufidx);
}
