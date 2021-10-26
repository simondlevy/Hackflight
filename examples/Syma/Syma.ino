/*
   Hackflight sketch for Ladybug Flight Controller with Spektrum DSMX receiver

   Additional libraries needed:

       https://github.com/simondlevy/USFS
       https://github.com/simondlevy/CrossPlatformDataBus
       https://github.com/simondlevy/SpektrumDSM 

   Hardware support for Ladybug flight controller:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "copilot.h"

#include "stream_serial.h"
#include "parser.hpp"
#include "debugger.hpp"
#include "stream_receiver.h"

void stream_writeBrushedMotors(
        uint8_t m1_pin, uint8_t m2_pin, uint8_t m3_pin, uint8_t m4_pin,
        float m1_val, float m2_val, float m3_val, float m4_val);

static void setOutBuf(uint8_t * buffer, uint8_t index, uint8_t byte)
{
    buffer[index] = byte;
}

static void serialize(uint8_t * buffer, uint8_t & buffer_size, uint8_t & crc_out, uint8_t byte)
{
    setOutBuf(buffer, buffer_size, byte);
    buffer_size = buffer_size + 1;
    crc_out ^= byte;
}

static void serializeFloat(uint8_t * buffer, uint8_t & buffer_size, uint8_t & crc_out, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(buffer, buffer_size, crc_out, uintval     & 0xFF);
    serialize(buffer, buffer_size, crc_out, uintval>>8  & 0xFF);
    serialize(buffer, buffer_size, crc_out, uintval>>16 & 0xFF);
    serialize(buffer, buffer_size, crc_out, uintval>>24 & 0xFF);
}


void setup(void)
{
}

void loop(void)
{
    step();
}

void stream_run(
        float state_phi,
        float state_theta,
        float state_psi,
        bool armed,
        uint8_t m1_pin,
        uint8_t m2_pin,
        uint8_t m3_pin,
        uint8_t m4_pin,
        float m1_flying,
        float m2_flying,
        float m3_flying,
        float m4_flying,
        bool serialAvail,
        uint8_t serialByte)
{
    static uint8_t _outbuff[128];
    static uint8_t _inbuff[128];
    static uint8_t _outbuff_size;
    static uint8_t _outbuff_index;
    static uint8_t _crc_out;

    bool sending = false;
    bool receiving = false;
    uint8_t index = 0;
    uint8_t msgtype = 0;

    parse(serialAvail, serialByte, sending, receiving, index, msgtype);

    if (receiving) {
        _inbuff[index] = serialByte;
    }

    if (sending) {

        setOutBuf(_outbuff, 0, '$');
        setOutBuf(_outbuff, 1, 'M');
        setOutBuf(_outbuff, 2, '>');

        _outbuff_index = 0;

        uint8_t outsize = 4 * (msgtype == 121 ? 6 : msgtype == 122 ? 3 : 0);

        _outbuff[3] = outsize;
        _outbuff[4] = msgtype;

        _outbuff_size = 5;
        _crc_out = outsize ^ msgtype;

        serializeFloat(_outbuff, _outbuff_size, _crc_out,
                msgtype == 121 ? stream_receiverThrottle : msgtype == 122 ? state_phi : 0);

        serializeFloat(_outbuff, _outbuff_size, _crc_out,
                msgtype == 121 ? stream_receiverRoll : msgtype == 122 ? state_theta : 0);

        serializeFloat(_outbuff, _outbuff_size, _crc_out,
                msgtype == 121 ? stream_receiverPitch : msgtype == 122 ? state_psi : 0);

        if (msgtype == 121) {
            serializeFloat(_outbuff, _outbuff_size, _crc_out, stream_receiverYaw);
            serializeFloat(_outbuff, _outbuff_size, _crc_out, stream_receiverAux1);
            serializeFloat(_outbuff, _outbuff_size, _crc_out, stream_receiverAux2);
        }

        serialize(_outbuff, _outbuff_size, _crc_out, _crc_out);
    }

    uint8_t motor_index = msgtype == 215 ? _inbuff[0] : 0;
    uint8_t motor_percent = msgtype == 215 ? _inbuff[1] : 0;

    if (_outbuff_size > 0) {
        _outbuff_size = _outbuff_size - 1;
        uint8_t data_byte = _outbuff[_outbuff_index];
        _outbuff_index = _outbuff_index + 1;
        stream_serialWrite(data_byte);
    }

    float m1_val = armed ? m1_flying : motor_index == 1 ? motor_percent/100. : 0;
    float m2_val = armed ? m2_flying : motor_index == 2 ? motor_percent/100. : 0;
    float m3_val = armed ? m3_flying : motor_index == 3 ? motor_percent/100. : 0;
    float m4_val = armed ? m4_flying : motor_index == 4 ? motor_percent/100. : 0;

    stream_writeBrushedMotors(m1_pin, m2_pin, m3_pin, m4_pin, m1_val, m2_val, m3_val, m4_val);
}
