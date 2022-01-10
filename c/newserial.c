/*
   Platform-independent support for serial comms that can't be done in Haskell

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <stdint.h>
#include <string.h>

static void serializeByte(uint8_t * buf, uint8_t idx, uint8_t & crc, uint8_t byte)
{
    buf[idx] = byte;
    crc ^= byte;
}

static void serializeFloat(uint8_t * buf, uint8_t index, uint8_t & crc, float value)
{
    uint32_t uintval = 0;

    memcpy(&uintval, &value, 4);

    uint8_t k = 4 * index + 5;

    serializeByte(buf, k,   crc, uintval     & 0xFF);
    serializeByte(buf, k+1, crc, uintval>>8  & 0xFF);
    serializeByte(buf, k+2, crc, uintval>>16 & 0xFF);
    serializeByte(buf, k+3, crc, uintval>>24 & 0xFF);
}

uint8_t serializeMessage(
          uint8_t buf[256]
        , uint8_t hdr0
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
    buf[0] = hdr0;
    buf[1] = hdr1;
    buf[2] = hdr2;
    buf[3] = hdr3;
    buf[4] = hdr4;

    if (size > 0) serializeFloat(buf, 0, crc, val00);
    if (size > 1) serializeFloat(buf, 1, crc, val01);
    if (size > 2) serializeFloat(buf, 2, crc, val02);
    if (size > 3) serializeFloat(buf, 3, crc, val03);
    if (size > 4) serializeFloat(buf, 4, crc, val04);
    if (size > 5) serializeFloat(buf, 5, crc, val05);

    uint8_t last = 4 * size + 5;

    buf[last] = crc;

    return last + 1;
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
    void serialWrite(uint8_t * buf, uint8_t size);

    uint8_t buf[256] = {};

    uint8_t bufsize = serializeMessage(
          buf
        , hdr0
        , hdr1
        , hdr2
        , hdr3
        , hdr4
        , crc
        , size
        , val00
        , val01
        , val02
        , val03
        , val04
        , val05);

    serialWrite(buf, size);
}
