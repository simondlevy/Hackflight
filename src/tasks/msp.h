/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "core/vstate.h"
#include "debug.h"
#include "esc.h"
#include "imu.h"
#include "maths.h"
#include "receiver.h"
#include "system.h"
#include "task.h"

#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

class Msp : public Task {
    
    friend class Hackflight;

    private:

        static const uint8_t MAX_PORT_COUNT = 3;
        static const uint8_t PORT_INBUF_SIZE = 192;
        static const uint16_t PORT_DATAFLASH_BUFFER_SIZE = 4096;
        static const uint8_t PORT_DATAFLASH_INFO_SIZE = 16;
        static const uint16_t PORT_OUTBUF_SIZE =
            PORT_DATAFLASH_BUFFER_SIZE + PORT_DATAFLASH_INFO_SIZE;
        static const uint8_t JUMBO_FRAME_SIZE_LIMIT = 255;

        static const uint8_t RC        = 105;
        static const uint8_t ATTITUDE  = 108;    
        static const uint8_t SET_MOTOR = 214;    

        static int16_t rad2degi(float rad)
        {
            return (int16_t)Math::rad2deg(rad);
        }

        Esc *                m_esc;
        Arming *             m_arming;
        Receiver *           m_receiver;
        VehicleState *       m_vstate;

        float  motors[MAX_SUPPORTED_MOTORS];

        typedef struct sbuf_s {
            uint8_t *ptr;  
            uint8_t *end;
        } sbuf_t;

        static void sbufWriteU8(sbuf_t *dst, uint8_t val)
        {
            *dst->ptr++ = val;
        }

        static void sbufWriteU16(sbuf_t *dst, uint16_t val)
        {
            sbufWriteU8(dst, val >> 0);
            sbufWriteU8(dst, val >> 8);
        }

        static uint8_t sbufReadU8(sbuf_t *src)
        {
            return *src->ptr++;
        }

        static uint16_t sbufReadU16(sbuf_t *src)
        {
            uint16_t ret;
            ret = sbufReadU8(src);
            ret |= sbufReadU8(src) << 8;
            return ret;
        }

        // reader - return bytes remaining in buffer
        // writer - return available space
        static int sbufBytesRemaining(sbuf_t *buf)
        {
            return buf->end - buf->ptr;
        }

        static uint8_t* sbufPtr(sbuf_t *buf)
        {
            return buf->ptr;
        }

        // modifies streambuf so that written data are prepared for reading
        static void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
        {
            buf->end = buf->ptr;
            buf->ptr = base;
        }

        // return positive for ACK, negative on error, zero for no reply
        typedef enum {
            RESULT_ACK = 1,
            RESULT_ERROR = -1,
            RESULT_NO_REPLY = 0,
            RESULT_CMD_UNKNOWN = -2,   
        } mspResult_e;

        typedef enum {
            DIRECTION_REPLY = 0,
            DIRECTION_REQUEST = 1
        } mspDirection_e;


        typedef struct mspPacket_s {
            sbuf_t buf;         // payload only w/o header or crc
            int16_t cmd;
            int16_t result;
            uint8_t direction;  // It also looks like unused and might be deleted.
        } mspPacket_t;

        typedef int mspDescriptor_t;

        // msp post process function, used for gracefully handling reboots, etc.
        typedef void (*mspPostProcessFnPtr)(void * port); 

        typedef enum {
            IDLE,
            HEADER_START,
            HEADER_M,
            HEADER,
            PAYLOAD,
            CHECKSUM,
            COMMAND_RECEIVED
        } mspVehicleState_e;


        typedef enum {
            PENDING_NONE,
            PENDING_BOOTLOADER_ROM,
            PENDING_CLI,
            PENDING_BOOTLOADER_FLASH,
        } mspPendingSystemRequest_e;

        typedef enum {
            PACKET_COMMAND,
            PACKET_REPLY
        } mspPacketType_e;

        typedef struct mspPort_s {
            void *                    port; // null when port unused.
            uint32_t                  lastActivityMs;
            mspPendingSystemRequest_e pendingRequest;
            mspVehicleState_e                state;
            mspPacketType_e           packetType;
            uint8_t                   inBuf[PORT_INBUF_SIZE];
            uint16_t                  cmdMSP;
            uint_fast16_t             offset;
            uint_fast16_t             dataSize;
            uint8_t                   checksum;
        } mspPort_t;

        typedef struct __attribute__((packed)) {
            uint8_t size;
            uint8_t cmd;
        } mspHeadert;

        typedef struct __attribute__((packed)) {
            uint16_t size;
        } mspHeaderJUMBO_t;

        void *    m_dbgPort = NULL;
        bool      m_debugging = false;
        mspPort_t m_ports[MAX_PORT_COUNT];

        static float scale(const float value)
        {
            return 1000 + 1000 * value;
        }

        bool processOutCommand(int16_t cmdMSP, sbuf_t *dst)
        {
            auto unsupportedCommand = false;

            switch (cmdMSP) {

                case RC:
                    sbufWriteU16(dst, m_receiver->getRawThrottle());
                    sbufWriteU16(dst, m_receiver->getRawRoll());
                    sbufWriteU16(dst, m_receiver->getRawPitch());
                    sbufWriteU16(dst, m_receiver->getRawYaw());
                    sbufWriteU16(dst, scale(m_receiver->getRawAux1()));
                    sbufWriteU16(dst, scale(m_receiver->getRawAux2()));
                    break;

                case ATTITUDE:
                    sbufWriteU16(dst, 10 * rad2degi(m_vstate->phi));
                    sbufWriteU16(dst, 10 * rad2degi(m_vstate->theta));
                    sbufWriteU16(dst, rad2degi(m_vstate->psi));
                    break;

                default:
                    unsupportedCommand = true;
            }
            return !unsupportedCommand;
        }


        mspResult_e processInCommand(int16_t cmdMSP, sbuf_t *src, float * motors)
        {
            switch (cmdMSP) {

                case SET_MOTOR:
                    for (uint8_t i=0; i<m_esc->m_pins->size(); i++) {
                        motors[i] = m_esc->convertFromExternal(sbufReadU16(src));
                    }
                    break;

                default:
                    // we do not know how to handle the (valid) message, indicate error
                    // MSP $M!
                    return RESULT_ERROR;
            }
            return RESULT_ACK;
        }

        /*
         * Returns RESULT_ACK, RESULT_ERROR or RESULT_NO_REPLY
         */
        mspResult_e fcProcessCommand(mspPacket_t *cmd, mspPacket_t *reply, float * motors) {

            sbuf_t *dst = &reply->buf;
            sbuf_t *src = &cmd->buf;
            const auto cmdMSP = cmd->cmd;
            reply->cmd = cmd->cmd;

            mspResult_e ret = 
                processOutCommand(cmdMSP, dst) ?
                RESULT_ACK :
                processInCommand(cmdMSP, src, motors);

            reply->result = ret;

            return ret;
        }

        static void fcProcessReply(mspPacket_t *reply)
        {
            (void)reply;
        }

        static bool processReceivedData(mspPort_t *mspPort, uint8_t c)
        {
            switch (mspPort->state) {
                default:
                case IDLE:      // Waiting for '$' character
                    if (c == '$') {
                        mspPort->state = HEADER_START;
                    } else {
                        return false;
                    }
                    break;

                case HEADER_START: 
                    mspPort->offset = 0;
                    mspPort->checksum = 0;
                    switch (c) {
                        case 'M':
                            mspPort->state = HEADER_M;
                            break;
                        default:
                            mspPort->state = IDLE;
                            break;
                    }
                    break;

                case HEADER_M:      // Waiting for '<' / '>'
                    mspPort->state = HEADER;
                    switch (c) {
                        case '<':
                            mspPort->packetType = PACKET_COMMAND;
                            break;
                        case '>':
                            mspPort->packetType = PACKET_REPLY;
                            break;
                        default:
                            mspPort->state = IDLE;
                            break;
                    }
                    break;

                case HEADER:    
                    mspPort->inBuf[mspPort->offset++] = c;
                    mspPort->checksum ^= c;
                    if (mspPort->offset == sizeof(mspHeadert)) {

                        mspHeadert * hdr = (mspHeadert *)&mspPort->inBuf[0];

                        // Check incoming buffer size limit
                        if (hdr->size > PORT_INBUF_SIZE) {
                            mspPort->state = IDLE;
                        } else {
                            mspPort->dataSize = hdr->size;
                            mspPort->cmdMSP = hdr->cmd;
                            mspPort->offset = 0;                // re-use buffer
                            mspPort->state = mspPort->dataSize > 0 ?
                                PAYLOAD :
                                CHECKSUM;    // If no payload - jump to checksum byte
                        }
                    }
                    break;

                case PAYLOAD:
                    mspPort->inBuf[mspPort->offset++] = c;
                    mspPort->checksum ^= c;
                    if (mspPort->offset == mspPort->dataSize) {
                        mspPort->state = CHECKSUM;
                    }
                    break;

                case CHECKSUM:
                    if (mspPort->checksum == c) {
                        mspPort->state = COMMAND_RECEIVED;
                    } else {
                        mspPort->state = IDLE;
                    }
                    break;
            }

            return true;
        }

        static uint8_t mspSerialChecksumBuf(uint8_t checksum, const uint8_t *data, int len)
        {
            while (len-- > 0) {
                checksum ^= *data++;
            }
            return checksum;
        }

        static int serialSendFrame(
                mspPort_t *msp,
                const uint8_t * hdr,
                int hdrLen,
                const uint8_t * data,
                int dataLen,
                const uint8_t * crc,
                int crcLen)
        {
            const int totalFrameLength = hdrLen + dataLen + crcLen;

            // Transmit frame
            serialWriteBuf(msp->port, hdr, hdrLen);
            serialWriteBuf(msp->port, data, dataLen);
            serialWriteBuf(msp->port, crc, crcLen);

            return totalFrameLength;
        }

        static int serialEncode(mspPort_t *msp, mspPacket_t *packet)
        {
            const int dataLen = sbufBytesRemaining(&packet->buf);
            uint8_t hdrBuf[16] = { (uint8_t)'$', (uint8_t)'M', (uint8_t)(packet->result == RESULT_ERROR ? '!' : '>')};
            uint8_t crcBuf[2];
            uint8_t checksum;
            int hdrLen = 3;
            int crcLen = 0;

            mspHeadert * hdr = (mspHeadert *)&hdrBuf[hdrLen];
            hdrLen += sizeof(mspHeadert);
            hdr->cmd = packet->cmd;

            // Add JUMBO-frame header if necessary
            if (dataLen >= JUMBO_FRAME_SIZE_LIMIT) {
                mspHeaderJUMBO_t * hdrJUMBO = (mspHeaderJUMBO_t *)&hdrBuf[hdrLen];
                hdrLen += sizeof(mspHeaderJUMBO_t);

                hdr->size = JUMBO_FRAME_SIZE_LIMIT;
                hdrJUMBO->size = dataLen;
            } else {
                hdr->size = dataLen;
            }

            // Pre-calculate CRC
            checksum = mspSerialChecksumBuf(0, hdrBuf + 3, hdrLen - 3);
            checksum = mspSerialChecksumBuf(checksum, sbufPtr(&packet->buf), dataLen);
            crcBuf[crcLen++] = checksum;

            // Send the frame
            return serialSendFrame(msp, hdrBuf, hdrLen, sbufPtr(&packet->buf), dataLen,
                    crcBuf, crcLen);
        }

        mspPostProcessFnPtr processReceivedCommand(mspPort_t *msp, float * motors)
        {
            static uint8_t mspSerialOutBuf[PORT_OUTBUF_SIZE];

            mspPacket_t reply = {
                .buf = { .ptr = mspSerialOutBuf, .end = ARRAYEND(mspSerialOutBuf), },
                .cmd = -1,
                .result = 0,
                .direction = DIRECTION_REPLY,
            };
            auto *outBufHead = reply.buf.ptr;

            mspPacket_t command = {
                .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
                .cmd = (int16_t)msp->cmdMSP,
                .result = 0,
                .direction = DIRECTION_REQUEST,
            };

            mspPostProcessFnPtr mspPostProcessFn = NULL;

            const auto status = fcProcessCommand(&command, &reply, motors);

            if (status != RESULT_NO_REPLY) {
                sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
                serialEncode(msp, &reply);
            }

            return mspPostProcessFn;
        }

        static void evaluateNonMspData(mspPort_t * mspPort, uint8_t receivedChar)
        {
            if (receivedChar == 'R') {
                mspPort->pendingRequest = PENDING_BOOTLOADER_ROM;
            }
        }

        static void processPendingRequest(mspPort_t * mspPort)
        {
            // If no request is pending or 100ms guard time has not elapsed - do nothing
            if ((mspPort->pendingRequest == PENDING_NONE) ||
                    (timeMillis() - mspPort->lastActivityMs < 100)) {
                return;
            }

            switch(mspPort->pendingRequest) {
                case PENDING_BOOTLOADER_ROM:
                    systemReboot();
                    break;

                default:
                    break;
            }
        }

    public:

        void triggerDebugging(void)
        {
            m_debugging = true;
        }

    public:

        Msp() : Task(100) { } // Hz

        void begin(Esc * esc, Arming * arming, Receiver * receiver, VehicleState * vstate)
        {
            memset(m_ports, 0, sizeof(m_ports));

            m_esc = esc;

            m_arming = arming;

            m_ports[0].port = serialOpenPortUsb();

            m_dbgPort = m_ports[0].port; 

            serialDebugSetPort(m_dbgPort);

            m_vstate = vstate;

            m_receiver = receiver;
        }

        virtual void fun(uint32_t usec) override
        {
            (void)usec;

            // Sync debugging to MSP update
            if (m_debugging) {
                serialDebugFlush();
                m_debugging = false;
            }

            else for (auto portId=0; portId<MAX_PORT_COUNT; portId++) {

                mspPort_t * const mspPort = &m_ports[portId];

                if (!mspPort->port) {
                    continue;
                }

                mspPostProcessFnPtr mspPostProcessFn = NULL;

                if (serialBytesAvailable(mspPort->port)) {
                    // There are bytes incoming - abort pending request
                    mspPort->lastActivityMs = timeMillis();
                    mspPort->pendingRequest = PENDING_NONE;

                    while (serialBytesAvailable(mspPort->port)) {

                        const auto c = serialRead(mspPort->port);

                        const auto consumed = processReceivedData(mspPort, c);

                        if (!consumed && !m_arming->isArmed()) {
                            evaluateNonMspData(mspPort, c);
                        }

                        if (mspPort->state == COMMAND_RECEIVED) {
                            if (mspPort->packetType == PACKET_COMMAND) {
                                mspPostProcessFn = processReceivedCommand(mspPort, motors);
                            } else if (mspPort->packetType == PACKET_REPLY) {
                                mspPort->state = IDLE;
                            }

                            mspPort->state = IDLE;
                            break; 
                            // process one command at a time so as not to block.
                        }
                    }

                    if (mspPostProcessFn) {
                        mspPostProcessFn(mspPort->port);
                    }
                } else {
                    processPendingRequest(mspPort);
                }
            }

        }
};
