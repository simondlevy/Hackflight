/*
   msp.hpp : MSP (Multiwii Serial Protocol) support

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MEReceiverHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "board.hpp"
#include "receiver.hpp"
#include "mixer.hpp"

// See http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
#define MSP_RC_NORMAL            121    
#define MSP_ATTITUDE             108    
#define MSP_SET_MOTOR_NORMAL     215    

namespace hf {

static const int INBUF_SIZE = 128;

typedef enum serialState_t {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD
} serialState_t;

typedef  struct mspPortState_t {
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[INBUF_SIZE];
    uint8_t cmdMSP;
    uint8_t offset;
    uint8_t dataSize;
    serialState_t c_state;
} mspPortState_t;

class MSP {
public:
    void init(Mixer * _mixer, Receiver * _rc, Board * _board);
    void update(float eulerAngles[3], bool armed);

private:
    Mixer  * mixer;
    Receiver     * rc;
    Board  * board;

    mspPortState_t portState;

    void     serialize8(uint8_t a);
    void     serialize16(int16_t a);
    uint8_t  read8(void);
    uint16_t read16(void);
    uint32_t read32(void);
    float    readFloat(void);
    void     serializeFloat(float f);
    void     serialize32(uint32_t a);
    void     headSerialResponse(uint8_t err, uint8_t s);
    void     headSerialReply(uint8_t s);
    void     headSerialError(uint8_t s);
    void     tailSerialReply(void);

}; // class MSP


/********************************************* CPP ********************************************************/


void MSP::serialize8(uint8_t a)
{
    board->serialWriteByte(a);
    portState.checksum ^= a;
}

void MSP::serialize16(int16_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

uint8_t MSP::read8(void)
{
    return portState.inBuf[portState.indRX++] & 0xff;
}

uint16_t MSP::read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

uint32_t MSP::read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}

float MSP::readFloat(void)
{
    float f = 0;
    uint32_t t = read32();
    memcpy(&f, &t, 4);
    return f;
}

void MSP::serializeFloat(float f)
{
    uint32_t a;
    memcpy(&a, &f, 4);
    serialize32(a);
}

void MSP::serialize32(uint32_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}


void MSP::headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    portState.checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(portState.cmdMSP);
}

void MSP::headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void MSP::headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void MSP::tailSerialReply(void)
{
    serialize8(portState.checksum);
}

void MSP::init(Mixer * _mixer, Receiver * _rc, Board * _board)
{
    mixer = _mixer;
    rc    = _rc;
    board = _board;

    memset(&portState, 0, sizeof(portState));
}

void MSP::update(float eulerAngles[3], bool armed)
{
    while (board->serialAvailableBytes()) {

        uint8_t c = board->serialReadByte();

        if (portState.c_state == IDLE) {
            portState.c_state = (c == '$') ? HEADER_START : IDLE;
            if (portState.c_state == IDLE && !armed) {
            }
        } else if (portState.c_state == HEADER_START) {
            portState.c_state = (c == 'M') ? HEADER_M : IDLE;
        } else if (portState.c_state == HEADER_M) {
            portState.c_state = (c == '<') ? HEADER_ARROW : IDLE;
        } else if (portState.c_state == HEADER_ARROW) {
            if (c > INBUF_SIZE) {       // now we are expecting the payload size
                portState.c_state = IDLE;
                continue;
            }
            portState.dataSize = c;
            portState.offset = 0;
            portState.checksum = 0;
            portState.indRX = 0;
            portState.checksum ^= c;
            portState.c_state = HEADER_SIZE;      // the command is to follow
        } else if (portState.c_state == HEADER_SIZE) {
            portState.cmdMSP = c;
            portState.checksum ^= c;
            portState.c_state = HEADER_CMD;
        } else if (portState.c_state == HEADER_CMD && 
            portState.offset < portState.dataSize) {
            portState.checksum ^= c;
            portState.inBuf[portState.offset++] = c;
        } else if (portState.c_state == HEADER_CMD && portState.offset >= portState.dataSize) {

            if (portState.checksum == c) {        // compare calculated and transferred checksum

                switch (portState.cmdMSP) {

                case MSP_SET_MOTOR_NORMAL:
                    for (uint8_t i = 0; i < 4; i++)
                        mixer->motorsDisarmed[i] = readFloat();
                    headSerialReply(0);
                    break;

                case MSP_RC_NORMAL:
                    headSerialReply(32);
                    for (uint8_t i = 0; i < 8; i++)
                        serializeFloat(rc->raw[i]);
                    break;

                case MSP_ATTITUDE: 
                    headSerialReply(6);
                    for (uint8_t i = 0; i < 3; i++)
                        serialize16((int16_t)(eulerAngles[i]));
                    break;

                    // don't know how to handle the (valid) message, indicate error MSP $M!
                default:                   
                    headSerialError(0);
                    break;
                }
                tailSerialReply();
            }
            portState.c_state = IDLE;
        }
    }
}

} // namespace
