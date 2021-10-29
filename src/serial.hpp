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

static uint8_t _outbuff[128];
static uint8_t _outbuff_size;
static uint8_t _outbuff_index;

static void serialize(uint8_t & crc_out, uint8_t byte)
{
    _outbuff[_outbuff_size++] = byte;
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

void handleSerialInput(
        uint8_t msgtype
      , float state_phi
      , float state_theta
      , float state_psi)
{
    static uint8_t _crc_out;

    _outbuff_index = 0;

    _outbuff[0] = '$';
    _outbuff[1] = 'M';
    _outbuff[2] = '>';

    uint8_t outsize = 4 * (msgtype == 121 ? 6 : msgtype == 122 ? 3 : 0);

    _outbuff[3] = outsize;
    _outbuff[4] = msgtype;

    _outbuff_size = 5;
    _crc_out = outsize ^ msgtype;

    if (msgtype == 122) {
        serializeFloat(_crc_out, state_phi);
        serializeFloat(_crc_out, state_theta);
        serializeFloat(_crc_out, state_psi);
    }

    if (msgtype == 121) {
        serializeFloat(_crc_out, stream_receiverThrottle);
        serializeFloat(_crc_out, stream_receiverRoll);
        serializeFloat(_crc_out, stream_receiverPitch);
        serializeFloat(_crc_out, stream_receiverYaw);
        serializeFloat(_crc_out, stream_receiverAux1);
        serializeFloat(_crc_out, stream_receiverAux2);
    }

    serialize(_crc_out, _crc_out);
}

static void updateSerialOutput(void)
{
    if (_outbuff_size > 0) {
        _outbuff_size = _outbuff_size - 1;
        uint8_t data_byte = _outbuff[_outbuff_index];
        _outbuff_index = _outbuff_index + 1;
        stream_serialWrite(data_byte);
    }
}
