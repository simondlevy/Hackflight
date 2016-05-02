/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */


#include "mw.h"
#include "config.h"
#include "board.h"

// Multiwii Serial Protocol 0
#define MSP_STATUS               101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102    //out message         9 DOF
#define MSP_MOTOR                104    //out message         8 motors
#define MSP_RC                   105    //out message         8 rc chan and more
#define MSP_ATTITUDE             108    //out message         2 angles 1 heading
#define MSP_ALTITUDE             109    //out message         altitude, variometer

#define MSP_BARO_SONAR_RAW       126    // out message

#define MSP_SET_RAW_RC           200    //in message          8 rc chan
#define MSP_SET_MOTOR            214    //in message          PropBalance function


// Additional private MSP for baseflight configurator
#define MSP_REBOOT               68     //in message          reboot settings
#define MSP_BUILDINFO            69     //out message         build date as well as some space for future expansion

#define INBUF_SIZE 128

// from mixer.c
extern int16_t motorsDisarmed[4];

// cause reboot after MSP processing complete
static bool pendReboot = false;

typedef enum serialState_t {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
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

static mspPortState_t portState;

static bool rxMspFrameDone = false;

static void mspFrameReceive(void)
{
    rxMspFrameDone = true;
}

static void serialize8(uint8_t a)
{
    board_serialWrite(a);
    portState.checksum ^= a;
}

static void serialize16(int16_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

static void serialize32(uint32_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}

static uint8_t read8(void)
{
    return portState.inBuf[portState.indRX++] & 0xff;
}

static uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

/*
static uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}
*/

static void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    portState.checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(portState.cmdMSP);
}

static void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

static void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

static void tailSerialReply(void)
{
    serialize8(portState.checksum);
}

static void s_struct(uint8_t *cb, uint8_t siz)
{
    headSerialReply(siz);
    while (siz--)
        serialize8(*cb++);
}

static void evaluateCommand(void)
{
    uint32_t i;
    const char *build = __DATE__;

    switch (portState.cmdMSP) {

        case MSP_SET_RAW_RC:
            for (i = 0; i < 8; i++)
                rcData[i] = read16();
            headSerialReply(0);
            mspFrameReceive();
            break;

        case MSP_SET_MOTOR:
            for (i = 0; i < 4; i++)
                motorsDisarmed[i] = read16();
            headSerialReply(0);
            break;

        case MSP_STATUS:
            /*
            headSerialReply(11);
            serialize16(cycleTime);
            serialize16(board_getI2cErrorCounter());
            serialize16(0);
            serialize8(0);
            */
            break;

        case MSP_RAW_IMU:
            /*
            headSerialReply(18);
            if (acc1G > 1024) {
                for (i = 0; i < 3; i++)
                    serialize16(accSmooth[i] / 8);
            } else {
                for (i = 0; i < 3; i++)
                    serialize16(accSmooth[i]);
            }
            for (i = 0; i < 3; i++)
                serialize16(gyroADC[i]);
            for (i = 0; i < 3; i++)
                serialize16(magADC[i]);
                */
            break;

        case MSP_MOTOR:
            s_struct((uint8_t *)motors, 16);
            break;

        case MSP_RC:
            headSerialReply(16);
            for (i = 0; i < 8; i++)
                serialize16(rcData[i]);
            break;

        case MSP_ATTITUDE:
            headSerialReply(6);
            for (i = 0; i < 2; i++)
                serialize16(angle[i]);
            serialize16(heading);
            break;

        case MSP_BARO_SONAR_RAW:
            //headSerialReply(8);
            //serialize32(baroPressure);
            //serialize32(sonarDistance);
            break;

        case MSP_ALTITUDE:
            //headSerialReply(6);
            //serialize32(estAlt);
            //serialize16(vario);
            break;

        case MSP_REBOOT:
            headSerialReply(0);
            pendReboot = true;
            break;

        case MSP_BUILDINFO:
            headSerialReply(11 + 4 + 4);
            for (i = 0; i < 11; i++)
                serialize8(build[i]); // MMM DD YYYY as ascii, MMM = Jan/Feb... etc
            serialize32(0); // future exp
            serialize32(0); // future exp
            break;

        default:                   // we do not know how to handle the (valid) message, indicate error MSP $M!
            headSerialError(0);
            break;
    }
    tailSerialReply();
}

// ================================================================================================================

void mspCom(void)
{
    uint8_t c;

    board_checkReboot(pendReboot);

    while (board_serialAvailable()) {

        c = board_serialRead();

        if (portState.c_state == IDLE) {
            portState.c_state = (c == '$') ? HEADER_START : IDLE;
            if (portState.c_state == IDLE && !armed) {
                if (c == '#')
                    ;
                else if (c == CONFIG_REBOOT_CHARACTER) 
                    board_reboot();
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
        } else if (portState.c_state == HEADER_CMD && 
                portState.offset >= portState.dataSize) {
            if (portState.checksum == c) {        // compare calculated and transferred checksum
                evaluateCommand();      // we got a valid packet, evaluate it
            }
            portState.c_state = IDLE;
        }
    }
}
