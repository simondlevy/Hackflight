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

static void setOutBuf(uint8_t * buffer, uint8_t index, bool sending, uint8_t byte)
{
    buffer[index] = sending ? byte : buffer[index];
}

static void serialize(uint8_t * buffer, uint8_t & buffer_size, uint8_t & crc_out, bool sending, uint8_t byte)
{
    setOutBuf(buffer, buffer_size, sending, byte);
    buffer_size = sending ? buffer_size + 1 : buffer_size;
    crc_out = sending ? crc_out ^ byte : crc_out;
}

static void serializeFloat(uint8_t * buffer, 
        uint8_t & buffer_size, uint8_t & crc_out, bool sending, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(buffer, buffer_size, crc_out, sending, uintval     & 0xFF);
    serialize(buffer, buffer_size, crc_out, sending, uintval>>8  & 0xFF);
    serialize(buffer, buffer_size, crc_out, sending, uintval>>16 & 0xFF);
    serialize(buffer, buffer_size, crc_out, sending, uintval>>24 & 0xFF);
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
    static uint8_t _buffer[128];
    static uint8_t _buffer_size;
    static uint8_t _buffer_index;
    static uint8_t _crc_out;

    bool sending = false;
    bool receiving = false;
    uint8_t index = 0;
    uint8_t msgtype = 0;

    parse(serialAvail, serialByte, sending, receiving, index, msgtype);

    _buffer[index] = receiving ? serialByte : _buffer[index];

    _buffer_index = sending ? 0 : _buffer_index;

    setOutBuf(_buffer, 0, sending, '$');
    setOutBuf(_buffer, 1, sending, 'M');
    setOutBuf(_buffer, 2, sending, '>');

    uint8_t outsize = 4 * (msgtype == 121 ? 6 : msgtype == 122 ? 3 : 0);

    _buffer_size = sending ? 3 : _buffer_size;
    _buffer[_buffer_size] = sending ? outsize : _buffer[_buffer_size];

    _buffer_size = sending ? 4 : _buffer_size;
    _buffer[_buffer_size] = sending ? msgtype : _buffer[_buffer_size];

    _buffer_size = sending ? 5 : _buffer_size;

    _crc_out = sending ? outsize ^ msgtype : 0;

    serializeFloat(_buffer, _buffer_size, _crc_out, sending,
            msgtype == 121 ? stream_receiverThrottle : msgtype == 122 ? state_phi : 0);

    serializeFloat(_buffer, _buffer_size, _crc_out, sending,
            msgtype == 121 ? stream_receiverRoll : msgtype == 122 ? state_theta : 0);

    serializeFloat(_buffer, _buffer_size, _crc_out, sending,
            msgtype == 121 ? stream_receiverPitch : msgtype == 122 ? state_psi : 0);

    bool sendrc = sending && msgtype == 121;

    serializeFloat(_buffer, _buffer_size, _crc_out, sendrc, stream_receiverYaw);

    serializeFloat(_buffer, _buffer_size, _crc_out, sendrc, stream_receiverAux1);

    serializeFloat(_buffer, _buffer_size, _crc_out, sendrc, stream_receiverAux2);

    serialize(_buffer, _buffer_size, _crc_out, sending, _crc_out);

    uint8_t motor_index = msgtype == 215 ? _buffer[0] : 0;
    uint8_t motor_percent = msgtype == 215 ? _buffer[1] : 0;

    uint8_t data_available = _buffer_size > 0;

    _buffer_size = data_available ? _buffer_size - 1 : _buffer_size;

    uint8_t data_byte = data_available ? _buffer[_buffer_index] : 0;

    _buffer_index = data_available ? _buffer_index + 1 : _buffer_index;

    if (data_available) {
        stream_serialWrite(data_byte);
    }

    float m1_val = armed ? m1_flying : motor_index == 1 ? motor_percent/100. : 0;
    float m2_val = armed ? m2_flying : motor_index == 2 ? motor_percent/100. : 0;
    float m3_val = armed ? m3_flying : motor_index == 3 ? motor_percent/100. : 0;
    float m4_val = armed ? m4_flying : motor_index == 4 ? motor_percent/100. : 0;

    stream_writeBrushedMotors(m1_pin, m2_pin, m3_pin, m4_pin, m1_val, m2_val, m3_val, m4_val);
}
