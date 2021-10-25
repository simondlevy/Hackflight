/*
   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#include "parser.hpp"
#include "debugger.hpp"
#include "stream_serial.h"
#include "stream_receiver.h"
#include "copilot.h"

extern float stream_receiverRoll;
extern float stream_receiverPitch;
extern float stream_receiverYaw;

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
    Serial1.begin(115200);
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
        bool  sending_,
        bool  receiving_,
        uint8_t index_,
        uint8_t msgtype_)
{
    static uint8_t _buffer[128];
    static uint8_t _buffer_size;
    static uint8_t _buffer_index;
    static uint8_t _crc_out;

    bool sending = false;
    bool receiving = false;
    uint8_t index = 0;
    uint8_t msgtype = 0;

    parse(stream_serialAvailable, stream_serialByte,
          sending, receiving, index, msgtype);

    _buffer[index] = receiving ? stream_serialByte : _buffer[index];

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

    stream_serialUpdate();

    if (stream_serialAvailable) {
        stream_serialRead();
    }

    if (data_available) {
        stream_serialWrite(data_byte);
    }



    delay(10);
}
