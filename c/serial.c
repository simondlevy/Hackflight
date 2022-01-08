/*
   Platform-independent support for serial comms that can't be done in Haskell

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <stdint.h>
#include <string.h>

void serialWrite(uint8_t byte);

static void serializeByte(uint8_t & crc, uint8_t byte)
{
    serialWrite(byte);
    crc ^= byte;
}

static void serializeFloat(uint8_t & crc, float value)
{
    uint32_t uintval = 0;

    memcpy(&uintval, &value, 4);

    serializeByte(crc, uintval     & 0xFF);
    serializeByte(crc, uintval>>8  & 0xFF);
    serializeByte(crc, uintval>>16 & 0xFF);
    serializeByte(crc, uintval>>24 & 0xFF);
}

void serialSend(
          uint8_t hdr0
        , uint8_t hdr1
        , uint8_t hdr2
        , uint8_t hdr3
        , uint8_t hdr4
        , uint8_t crc
        , uint8_t size
        , float val00
        , float val01
        , float val02
        , float val03
        , float val04
        , float val05
        )
{
    serialWrite(hdr0);
    serialWrite(hdr1);
    serialWrite(hdr2);
    serialWrite(hdr3);
    serialWrite(hdr4);

    if (size > 0) serializeFloat(crc, val00);
    if (size > 1) serializeFloat(crc, val01);
    if (size > 2) serializeFloat(crc, val02);
    if (size > 3) serializeFloat(crc, val03);
    if (size > 4) serializeFloat(crc, val04);
    if (size > 5) serializeFloat(crc, val05);

    serializeByte(crc, crc);
}
