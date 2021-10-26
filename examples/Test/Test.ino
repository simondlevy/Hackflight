/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "parser.hpp"
#include "debugger.hpp"
#include "stream_serial.h"
#include "stream_receiver.h"
#include "copilot.h"

//uint8_t stream_serialByte;

extern float stream_receiverRoll;
extern float stream_receiverPitch;
extern float stream_receiverYaw;

static void setOutBuf(uint8_t * buffer, uint8_t index, uint8_t byte)
{
    buffer[index] = byte;
}

static void serialize(uint8_t * buffer, uint8_t & buffer_size, uint8_t & crc_out, uint8_t byte)
{
    setOutBuf(buffer, buffer_size, byte);
    buffer_size = buffer_size + 1;
    crc_out = crc_out ^ byte;
}

static void serializeFloat(uint8_t * buffer, 
        uint8_t & buffer_size, uint8_t & crc_out, float value)
{
    uint32_t uintval = 1000 * (value + 2);

    serialize(buffer, buffer_size, crc_out, uintval     & 0xFF);
    serialize(buffer, buffer_size, crc_out, uintval>>8  & 0xFF);
    serialize(buffer, buffer_size, crc_out, uintval>>16 & 0xFF);
    serialize(buffer, buffer_size, crc_out, uintval>>24 & 0xFF);
}


void setup(void)
{
    Serial1.begin(115200);

    for (uint8_t k=0; k<10; ++k) {
        Serial1.println();
    }
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
        bool sending)
{
    static uint8_t _buffer[128];
    static uint8_t _buffer_size;
    static uint8_t _buffer_index;
    static uint8_t _crc_out;

    if (sending) {

        uint8_t msgtype = 122;

        Debugger::printf(Serial1, "msgtype: %d\n", msgtype);

        _buffer_index = 0;

        setOutBuf(_buffer, 0, '$');
        setOutBuf(_buffer, 1, 'M');
        setOutBuf(_buffer, 2, '>');

        uint8_t outsize = 4 * (msgtype == 121 ? 6 : msgtype == 122 ? 3 : 0);

        _buffer_size = 3;
        _buffer[_buffer_size] = outsize;

        _buffer_size = 4; 
        _buffer[_buffer_size] = msgtype;

        _buffer_size = 5;

        _crc_out = outsize ^ msgtype;

        serializeFloat(_buffer, _buffer_size, _crc_out, 
                msgtype == 121 ? stream_receiverThrottle : msgtype == 122 ? state_phi : 0);

        serializeFloat(_buffer, _buffer_size, _crc_out, 
                msgtype == 121 ? stream_receiverRoll : msgtype == 122 ? state_theta : 0);

        serializeFloat(_buffer, _buffer_size, _crc_out, 
                msgtype == 121 ? stream_receiverPitch : msgtype == 122 ? state_psi : 0);

        bool sendrc = msgtype == 121;

        if (msgtype == 121) {

            serializeFloat(_buffer, _buffer_size, _crc_out, stream_receiverYaw);
            serializeFloat(_buffer, _buffer_size, _crc_out, stream_receiverAux1);
            serializeFloat(_buffer, _buffer_size, _crc_out, stream_receiverAux2);
        }

        serialize(_buffer, _buffer_size, _crc_out, _crc_out);

        bool data_available = _buffer_size > 0;

        _buffer_size = data_available ? _buffer_size - 1 : _buffer_size;

        uint8_t data_byte = data_available ? _buffer[_buffer_index] : 0;

        _buffer_index = data_available ? _buffer_index + 1 : _buffer_index;

        if (data_available) {
            stream_serialWrite(data_byte);
        }
    }

    delay(10);
}
