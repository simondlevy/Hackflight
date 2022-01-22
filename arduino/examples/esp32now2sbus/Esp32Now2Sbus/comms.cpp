/*
   Platform-independent support for serial comms that can't be done in Haskell

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <stdint.h>
#include <string.h>

static uint8_t buffer[256];
static uint8_t bufidx;

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

void serialSend(
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

    serializeByte(crc, crc);

    void commsWrite(uint8_t *, uint8_t);
    commsWrite(buffer, bufidx);
}
