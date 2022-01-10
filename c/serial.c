/*
   Platform-independent support for serial comms that can't be done in Haskell

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <stdint.h>
#include <string.h>

void serialWrite(uint8_t byte);

static void serializeByte(void (*sendfun)(uint8_t), uint8_t & crc, uint8_t byte)
{
    sendfun(byte);
    crc ^= byte;
}

static void serializeFloat(void (*sendfun)(uint8_t), uint8_t & crc, float value)
{
    uint32_t uintval = 0;

    memcpy(&uintval, &value, 4);

    serializeByte(sendfun, crc, uintval     & 0xFF);
    serializeByte(sendfun, crc, uintval>>8  & 0xFF);
    serializeByte(sendfun, crc, uintval>>16 & 0xFF);
    serializeByte(sendfun, crc, uintval>>24 & 0xFF);
}

void serializeBytes(
          void (*sendfun)(uint8_t)
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
    sendfun(hdr0);
    sendfun(hdr1);
    sendfun(hdr2);
    sendfun(hdr3);
    sendfun(hdr4);

    if (size > 0) serializeFloat(sendfun, crc, val00);
    if (size > 1) serializeFloat(sendfun, crc, val01);
    if (size > 2) serializeFloat(sendfun, crc, val02);
    if (size > 3) serializeFloat(sendfun, crc, val03);
    if (size > 4) serializeFloat(sendfun, crc, val04);
    if (size > 5) serializeFloat(sendfun, crc, val05);

    serializeByte(crc, crc);
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
    serializeBytes(
            serialWrite,
            hdr0,
            hdr1,
            hdr2,
            hdr3,
            hdr4,
            crc,
            size,
            val00,
            val01,
            val02,
            val03,
            val04,
            val05);

}
