/*
   Hackflight stream-based serial support

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include <Arduino.h>

#include <string.h>

#include "stream_serial.h"

extern uint8_t stream_serialByte; 
extern bool stream_serialAvailable; 

static void serialize(uint8_t & crc, uint8_t byte)
{
    stream_serialWrite(byte);
    crc ^= byte;
}

static void serializeFloat(uint8_t & crc, float value)
{
    uint32_t uintval = 0;

    memcpy(&uintval, &value, 4);

    serialize(crc, uintval     & 0xFF);
    serialize(crc, uintval>>8  & 0xFF);
    serialize(crc, uintval>>16 & 0xFF);
    serialize(crc, uintval>>24 & 0xFF);
}


void stream_startSerial(void)
{
    Serial.begin(115200);
}

void stream_serialWrite(uint8_t byte)
{
    Serial.write(byte);
}

void stream_serialRead(void)
{
    stream_serialByte = Serial.read();
}

void stream_serialUpdate(void)
{
    stream_serialAvailable = Serial.available();
}

void stream_serialSend(
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
    stream_serialWrite(hdr0);
    stream_serialWrite(hdr1);
    stream_serialWrite(hdr2);
    stream_serialWrite(hdr3);
    stream_serialWrite(hdr4);

    if (size > 0) serializeFloat(crc, val00);
    if (size > 1) serializeFloat(crc, val01);
    if (size > 2) serializeFloat(crc, val02);
    if (size > 3) serializeFloat(crc, val03);
    if (size > 4) serializeFloat(crc, val04);
    if (size > 5) serializeFloat(crc, val05);

    serialize(crc, crc);
}
