/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "datatypes.h"
#include "debug.h"
#include "motor.h"
#include "rad2deg.h"
#include "serial.h"
#include "system.h"
#include "time.h"

#define MAX_MSP_PORT_COUNT 3
#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 4096
#define MSP_PORT_DATAFLASH_INFO_SIZE 16
#define MSP_PORT_OUTBUF_SIZE (MSP_PORT_DATAFLASH_BUFFER_SIZE + MSP_PORT_DATAFLASH_INFO_SIZE)
#define JUMBO_FRAME_SIZE_LIMIT 255

#define MSP_RC        105    
#define MSP_ATTITUDE  108    
#define MSP_SET_MOTOR 214    

#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])
#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))

static int16_t rad2degi(float rad)
{
    return (int16_t)rad2deg(rad);
}

// streambuf ------------------------------------------------------------------

typedef struct sbuf_s {
    uint8_t *ptr;          // data pointer must be first (sbuf_t* is equivalent to uint8_t **)
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

// ----------------------------------------------------------------------------

// return positive for ACK, negative on error, zero for no reply
typedef enum {
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0,
    MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
} mspResult_e;

typedef enum {
    MSP_DIRECTION_REPLY = 0,
    MSP_DIRECTION_REQUEST = 1
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

typedef mspResult_e (*mspProcessCommandFnPtr)(
        //mspDescriptor_t srcDesc,
        mspPacket_t *cmd,
        mspPacket_t *reply,
        mspPostProcessFnPtr *mspPostProcessFn,
        vehicle_state_t * vstate,
        rx_axes_t * rxax,
        float * motors);

typedef void (*mspProcessReplyFnPtr)(mspPacket_t *cmd);

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER,
    MSP_PAYLOAD,
    MSP_CHECKSUM,
    MSP_COMMAND_RECEIVED
} mspState_e;


typedef enum {
    MSP_PENDING_NONE,
    MSP_PENDING_BOOTLOADER_ROM,
    MSP_PENDING_CLI,
    MSP_PENDING_BOOTLOADER_FLASH,
} mspPendingSystemRequest_e;

typedef enum {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
} mspPacketType_e;

typedef struct mspPort_s {
    void *                    port; // null when port unused.
    uint32_t                  lastActivityMs;
    mspPendingSystemRequest_e pendingRequest;
    mspState_e                state;
    mspPacketType_e           packetType;
    uint8_t                   inBuf[MSP_PORT_INBUF_SIZE];
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

static bool processOutCommand(
        int16_t cmdMSP,
        sbuf_t *dst,
        vehicle_state_t * vstate,
        rx_axes_t * rxax)
{
    bool unsupportedCommand = false;

    switch (cmdMSP) {

        case MSP_RC:
            sbufWriteU16(dst, rxax->demands.throttle); 
            sbufWriteU16(dst, rxax->demands.roll);
            sbufWriteU16(dst, rxax->demands.pitch);
            sbufWriteU16(dst, rxax->demands.yaw);
            sbufWriteU16(dst, rxax->aux1);
            sbufWriteU16(dst, rxax->aux2);
            break;

        case MSP_ATTITUDE:
            sbufWriteU16(dst, 10 * rad2degi(vstate->phi));
            sbufWriteU16(dst, 10 * rad2degi(vstate->theta));
            sbufWriteU16(dst, rad2degi(vstate->psi));
            break;

        default:
            unsupportedCommand = true;
    }
    return !unsupportedCommand;
}


static mspResult_e processInCommand(int16_t cmdMSP, sbuf_t *src, float * motors)
{
    switch (cmdMSP) {

        case MSP_SET_MOTOR:
            for (int i = 0; i < 4; i++) {
                motors[i] = motorConvertFromExternal(sbufReadU16(src));
            }
            break;

        default:
            // we do not know how to handle the (valid) message, indicate error MSP $M!
            return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
static mspResult_e fcProcessCommand(
        //mspDescriptor_t srcDesc,
        mspPacket_t *cmd,
        mspPacket_t *reply,
        mspPostProcessFnPtr *mspPostProcessFn,
        vehicle_state_t * vstate,
        rx_axes_t * rxax,
        float * motors) {

    //(void)srcDesc;
    (void)mspPostProcessFn;

    int ret = MSP_RESULT_ACK;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const int16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (processOutCommand(cmdMSP, dst, vstate, rxax)) {
        ret = MSP_RESULT_ACK;
    } 
    else {
        ret = processInCommand(cmdMSP, src, motors);
    }
    reply->result = ret;
    return ret;
}

static void fcProcessReply(mspPacket_t *reply)
{
    (void)reply;
}

static mspPort_t mspPorts[MAX_MSP_PORT_COUNT];

static void * dbgPort = NULL;

static void resetPort(mspPort_t *mspPortToReset, void * serialPort)
{
    memset(mspPortToReset, 0, sizeof(mspPort_t));
    mspPortToReset->port = serialPort;
}

static void serialAllocatePort(void)
{
    void * serialPort = serialOpenPortUsb();

    resetPort(&mspPorts[0], serialPort);

    dbgPort = serialPort;
}

static bool serialProcessReceivedData(mspPort_t *mspPort, uint8_t c)
{
    switch (mspPort->state) {
        default:
        case MSP_IDLE:      // Waiting for '$' character
            if (c == '$') {
                mspPort->state = MSP_HEADER_START;
            } else {
                return false;
            }
            break;

        case MSP_HEADER_START: 
            mspPort->offset = 0;
            mspPort->checksum = 0;
            switch (c) {
                case 'M':
                    mspPort->state = MSP_HEADER_M;
                    break;
                default:
                    mspPort->state = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER_M:      // Waiting for '<' / '>'
            mspPort->state = MSP_HEADER;
            switch (c) {
                case '<':
                    mspPort->packetType = MSP_PACKET_COMMAND;
                    break;
                case '>':
                    mspPort->packetType = MSP_PACKET_REPLY;
                    break;
                default:
                    mspPort->state = MSP_IDLE;
                    break;
            }
            break;

        case MSP_HEADER:    
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum ^= c;
            if (mspPort->offset == sizeof(mspHeadert)) {

                mspHeadert * hdr = (mspHeadert *)&mspPort->inBuf[0];

                // Check incoming buffer size limit
                if (hdr->size > MSP_PORT_INBUF_SIZE) {
                    mspPort->state = MSP_IDLE;
                } else {
                    mspPort->dataSize = hdr->size;
                    mspPort->cmdMSP = hdr->cmd;
                    mspPort->offset = 0;                // re-use buffer
                    mspPort->state = mspPort->dataSize > 0 ?
                        MSP_PAYLOAD :
                        MSP_CHECKSUM;    // If no payload - jump to checksum byte
                }
            }
            break;

        case MSP_PAYLOAD:
            mspPort->inBuf[mspPort->offset++] = c;
            mspPort->checksum ^= c;
            if (mspPort->offset == mspPort->dataSize) {
                mspPort->state = MSP_CHECKSUM;
            }
            break;

        case MSP_CHECKSUM:
            if (mspPort->checksum == c) {
                mspPort->state = MSP_COMMAND_RECEIVED;
            } else {
                mspPort->state = MSP_IDLE;
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
    // We are allowed to send out the response if a) TX buffer is completely
    // empty (we are talking to well-behaving party that follows
    // request-response scheduling; this allows us to transmit jumbo frames
    // bigger than TX buffer (serialWriteBuf will block, but for jumbo frames
    // we don't care) b) Response fits into TX buffer
    const int totalFrameLength = hdrLen + dataLen + crcLen;
    if (!serialIsTransmitBufferEmpty(msp->port) &&
            ((int)serialTxBytesFree(msp->port) < totalFrameLength)) {
        return 0;
    }

    // Transmit frame
    serialBeginWrite(msp->port);
    serialWriteBuf(msp->port, hdr, hdrLen);
    serialWriteBuf(msp->port, data, dataLen);
    serialWriteBuf(msp->port, crc, crcLen);
    serialEndWrite(msp->port);

    return totalFrameLength;
}

static int serialEncode(mspPort_t *msp, mspPacket_t *packet)
{
    const int dataLen = sbufBytesRemaining(&packet->buf);
    uint8_t hdrBuf[16] = { '$', 'M', packet->result == MSP_RESULT_ERROR ? '!' : '>'};
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

static mspPostProcessFnPtr serialProcessReceivedCommand(
        mspPort_t *msp,
        mspProcessCommandFnPtr mspProcessCommandFn,
        vehicle_state_t * vstate,
        rx_axes_t * rxax,
        float * motors)
{
    static uint8_t mspSerialOutBuf[MSP_PORT_OUTBUF_SIZE];

    mspPacket_t reply = {
        .buf = { .ptr = mspSerialOutBuf, .end = ARRAYEND(mspSerialOutBuf), },
        .cmd = -1,
        .result = 0,
        .direction = MSP_DIRECTION_REPLY,
    };
    uint8_t *outBufHead = reply.buf.ptr;

    mspPacket_t command = {
        .buf = { .ptr = msp->inBuf, .end = msp->inBuf + msp->dataSize, },
        .cmd = msp->cmdMSP,
        .result = 0,
        .direction = MSP_DIRECTION_REQUEST,
    };

    mspPostProcessFnPtr mspPostProcessFn = NULL;
    const mspResult_e status = mspProcessCommandFn(
            &command,
            &reply,
            &mspPostProcessFn,
            vstate,
            rxax,
            motors);

    if (status != MSP_RESULT_NO_REPLY) {
        sbufSwitchToReader(&reply.buf, outBufHead); // change streambuf direction
        serialEncode(msp, &reply);
    }

    return mspPostProcessFn;
}

static void evaualteNonMspData(mspPort_t * mspPort, uint8_t receivedChar)
{
    if (receivedChar == 'R') {
        mspPort->pendingRequest = MSP_PENDING_BOOTLOADER_ROM;
    }
}

static void processPendingRequest(mspPort_t * mspPort)
{
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    if ((mspPort->pendingRequest == MSP_PENDING_NONE) ||
            (timeMillis() - mspPort->lastActivityMs < 100)) {
        return;
    }

    switch(mspPort->pendingRequest) {
        case MSP_PENDING_BOOTLOADER_ROM:
            systemReboot();
            break;

        default:
            break;
    }
}

static void serialProcessReceivedReply(mspPort_t *msp, mspProcessReplyFnPtr mspProcessReplyFn)
{
    mspPacket_t reply = {
        .buf = {
            .ptr = msp->inBuf,
            .end = msp->inBuf + msp->dataSize,
        },
        .cmd = msp->cmdMSP,
        .result = 0,
    };

    mspProcessReplyFn(&reply);

    msp->state = MSP_IDLE;
}

// ============================================================================================

static bool _debugging;

void mspInit(void)
{
    memset(mspPorts, 0, sizeof(mspPorts));
    serialAllocatePort();
    debugSetPort(dbgPort);
}

void mspTriggerDebugging(void)
{
    _debugging = true;
}

void mspUpdate(
        vehicle_state_t * vstate,
        rx_axes_t *rxaxes,
        bool armed,
        float * motors)
{
    // Sync debugging to MSP update
    if (_debugging) {
        debugFlush();
        _debugging = false;
    }

    else for (uint8_t portIndex = 0; portIndex < MAX_MSP_PORT_COUNT; portIndex++) {

        mspPort_t * const mspPort = &mspPorts[portIndex];

        if (!mspPort->port) {
            continue;
        }

        mspPostProcessFnPtr mspPostProcessFn = NULL;

        if (serialBytesAvailable(mspPort->port)) {
            // There are bytes incoming - abort pending request
            mspPort->lastActivityMs = timeMillis();
            mspPort->pendingRequest = MSP_PENDING_NONE;

            while (serialBytesAvailable(mspPort->port)) {
                const uint8_t c = serialRead(mspPort->port);
                const bool consumed = serialProcessReceivedData(mspPort, c);

                if (!consumed && !armed) {
                    evaualteNonMspData(mspPort, c);
                }

                if (mspPort->state == MSP_COMMAND_RECEIVED) {
                    if (mspPort->packetType == MSP_PACKET_COMMAND) {
                        mspPostProcessFn =
                            serialProcessReceivedCommand(
                                    mspPort,
                                    fcProcessCommand,
                                    vstate,
                                    rxaxes,
                                    motors);
                    } else if (mspPort->packetType == MSP_PACKET_REPLY) {
                        serialProcessReceivedReply(mspPort, fcProcessReply);
                    }

                    mspPort->state = MSP_IDLE;
                    break; // process one command at a time so as not to block.
                }
            }

            if (mspPostProcessFn) {
                serialWaitForPortToFinishTransmitting(mspPort->port);
                mspPostProcessFn(mspPort->port);
            }
        } else {
            processPendingRequest(mspPort);
        }
    }
}
