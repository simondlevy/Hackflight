/*
   Temporary C++ support for serial buffering until we get it working with Haskell 

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#pragma once

#include "stream_serial.h"

extern float stream_receiverThrottle;
extern float stream_receiverRoll;
extern float stream_receiverPitch;
extern float stream_receiverYaw;
extern float stream_receiverAux1;
extern float stream_receiverAux2;

static void serialize(uint8_t & crc_out, uint8_t byte)
{
    stream_serialWrite(byte);
    crc_out ^= byte;
}

static void serializeFloat(uint8_t & crc_out, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(crc_out, uintval     & 0xFF);
    serialize(crc_out, uintval>>8  & 0xFF);
    serialize(crc_out, uintval>>16 & 0xFF);
    serialize(crc_out, uintval>>24 & 0xFF);
}

void stream_handleSerialRequest(
        uint8_t msgtype
      , float state_phi
      , float state_theta
      , float state_psi)
{
    uint8_t crc_out = 0;

    stream_serialWrite('$');
    stream_serialWrite('M');
    stream_serialWrite('>');

    uint8_t outsize = 4 * (msgtype == 121 ? 6 : msgtype == 122 ? 3 : 0);

    stream_serialWrite(outsize);
    stream_serialWrite(msgtype);

    crc_out = outsize ^ msgtype;

    if (msgtype == 122) {
        serializeFloat(crc_out, state_phi);
        serializeFloat(crc_out, state_theta);
        serializeFloat(crc_out, state_psi);
    }

    if (msgtype == 121) {
        serializeFloat(crc_out, stream_receiverThrottle);
        serializeFloat(crc_out, stream_receiverRoll);
        serializeFloat(crc_out, stream_receiverPitch);
        serializeFloat(crc_out, stream_receiverYaw);
        serializeFloat(crc_out, stream_receiverAux1);
        serializeFloat(crc_out, stream_receiverAux2);
    }

    serialize(crc_out, crc_out);
}
