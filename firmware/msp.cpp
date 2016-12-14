/*
   msp.cpp : MSP (Multiwii Serial Protocol) class implementation

   Adapted from https://github.com/multiwii/baseflight/blob/master/src/serial.c

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef __arm__
extern "C" {
#else
#include <stdio.h>
#endif

#include <string.h> // for memset

#include "hackflight.hpp"

#define MSP_RC                   105    
#define MSP_ATTITUDE             108    
#define MSP_ALTITUDE             109    
#define MSP_BARO_SONAR_RAW       126    
#define MSP_SONARS               127    
#define MSP_SET_RAW_RC           200    
#define MSP_SET_HEAD             211
#define MSP_SET_MOTOR            214    

void MSP::serialize8(uint8_t a)
{
    Board::serialWriteByte(a);
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

void MSP::init(class IMU * _imu, class Hover * _hover, 
        class Mixer * _mixer, class RC * _rc, class Sonars * _sonars)
{
    this->imu = _imu;
    this->hover = _hover;
    this->mixer = _mixer;
    this->rc = _rc;
    this->sonars = _sonars;

    memset(&this->portState, 0, sizeof(this->portState));
}

void MSP::update(bool armed)
{
    while (Board::serialAvailableBytes()) {

        uint8_t c = Board::serialReadByte();

        if (portState.c_state == IDLE) {
            portState.c_state = (c == '$') ? HEADER_START : IDLE;
            if (portState.c_state == IDLE && !armed) {
                if (c == '#')
                    ;
                else if (c == CONFIG_REBOOT_CHARACTER) 
                    Board::reboot();
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

                    case MSP_SET_RAW_RC:
                        for (uint8_t i = 0; i < 8; i++)
                            this->rc->data[i] = read16();
                        headSerialReply(0);
                        break;

                    case MSP_SET_MOTOR:
                        for (uint8_t i = 0; i < 4; i++)
                            this->mixer->motorsDisarmed[i] = read16();
                        headSerialReply(0);
                        break;

                    case MSP_SET_HEAD: 
                        this->hover->headHold = read16();
                        headSerialReply(0);
                        break;

                    case MSP_RC:
                        headSerialReply(16);
                        for (uint8_t i = 0; i < 8; i++)
                            serialize16(this->rc->data[i]);
                        break;

                    case MSP_ATTITUDE:
                        headSerialReply(6);
                        for (uint8_t i = 0; i < 3; i++)
                            serialize16(this->imu->angle[i]);
                        break;

                    case MSP_BARO_SONAR_RAW:
                        //headSerialReply(8);
                        //serialize32(baroPressure);
                        //serialize32(sonarDistance);
                        break;

                    case MSP_ALTITUDE:
                        headSerialReply(6);
                        serialize32(this->hover->estAlt);
                        serialize16(this->hover->vario);
                        break;

                    case MSP_SONARS:
                        headSerialReply(8);
                        for (uint8_t i = 0; i < 4; i++)
                            serialize16(this->sonars->distances[i]);
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

#ifdef __arm__
} // extern "C"
#endif
