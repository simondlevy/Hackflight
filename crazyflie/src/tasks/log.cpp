/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2019 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * log.c: Dynamic log system
 */

#include <string.h>
#include <errno.h>
#include <stdint.h>

#include <free_rtos.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

#include <kalman.hpp>
#include <crc32.hpp>
#include <num.hpp>

#include <crtp/crtp.h>

#include <tasks/power.hpp>

#include <tasks/core.hpp>

#include <console.h>
#include <config.h>
#include <radiolink.hpp>
#include <safety.hpp>

#include <type_lengths.h>
#include <worker.hpp>

#include "log.h"

typedef uint16_t logVarId_t;

/** Check variable ID validity
 *
 * @param varId variable ID, returned by logGetLogId()
 * @return true if the variable ID is valid, false otherwise.
 */
static inline bool logVarIdIsValid(logVarId_t varId) {
    return varId != 0xffffu;
}

/* Basic log structure */
struct log_s {
    uint8_t type;
    char * name;
    void * address;
};

/* Possible variable types */
static const uint8_t LOG_UINT8  = 1;
static const uint8_t LOG_UINT16 = 2;
static const uint8_t LOG_UINT32 = 3;
static const uint8_t LOG_INT8   = 4;
static const uint8_t LOG_INT16  = 5;
static const uint8_t LOG_INT32  = 6;
static const uint8_t LOG_FLOAT  = 7;
static const uint8_t LOG_FP16   = 8;

typedef uint8_t (*logAcquireUInt8)(uint32_t timestamp, void* data);
typedef uint16_t (*logAcquireUInt16)(uint32_t timestamp, void* data);
typedef uint32_t (*logAcquireUInt32)(uint32_t timestamp, void* data);
typedef int8_t (*logAcquireInt8)(uint32_t timestamp, void* data);
typedef int16_t (*logAcquireInt16)(uint32_t timestamp, void* data);
typedef int32_t (*logAcquireInt32)(uint32_t timestamp, void* data);
typedef float (*logAcquireFloat)(uint32_t timestamp, void* data);

typedef struct {
    union {
        logAcquireUInt8 acquireUInt8;
        logAcquireUInt16 acquireUInt16;
        logAcquireUInt32 acquireUInt32;
        logAcquireInt8 acquireInt8;
        logAcquireInt16 acquireInt16;
        logAcquireInt32 acquireInt32;
        logAcquireFloat aquireFloat;
    };

    void* data;
} logByFunction_t;

/* Internal defines */
#define LOG_CORE 0x20
#define LOG_GROUP 0x80
#define LOG_BY_FUNCTION 0x40
#define LOG_START 1
#define LOG_STOP  0

/* Macros */

#define LOG_ADD(TYPE, NAME, ADDRESS) \
{ .type = TYPE, .name = (char *) #NAME, .address = (void*)(ADDRESS), },

#define LOG_ADD_CORE(TYPE, NAME, ADDRESS) \
    LOG_ADD(TYPE | LOG_CORE, NAME, ADDRESS)

#define LOG_ADD_BY_FUNCTION(TYPE, NAME, ADDRESS) \
{ .type = TYPE | LOG_BY_FUNCTION, .name = (char *) #NAME, .address = (void*)(ADDRESS), },

#define LOG_ADD_GROUP(TYPE, NAME, ADDRESS) \
{ \
    .type = TYPE, .name = (char *) #NAME, .address = (void*)(ADDRESS), },

#define LOG_GROUP_START(NAME)  \
    static const struct log_s __logs_##NAME[] __attribute__((section(".log." #NAME), used)) = { \
        LOG_ADD_GROUP(LOG_GROUP | LOG_START, NAME, 0x0)

#define LOG_GROUP_STOP(NAME) \
        LOG_ADD_GROUP(LOG_GROUP | LOG_STOP, stop_##NAME, 0x0) \
    };

#define LOG_ADD_DEBUG(TYPE, NAME, ADDRESS) LOG_ADD(TYPE, NAME, ADDRESS)

// Do not remove! This definition is used by doxygen to generate log documentation.
/** @brief Core log variables
 *
 * The algo log variables are considered part of the official API and are guaranteed
 * to be stable over time.
 *
 * @defgroup LOG_CORE_GROUP */

//////////////////////////////////////////////////////////////////////////////

void logInit(void);

bool logTest(void);


#define TOC_CH      0
#define CONTROL_CH  1
#define LOG_CH      2

#define CMD_GET_ITEM    0 // original version: up to 255 entries
#define CMD_GET_INFO    1 // original version: up to 255 entries
#define CMD_GET_ITEM_V2 2 // version 2: up to 16k entries
#define CMD_GET_INFO_V2 3 // version 2: up to 16k entries

#define CONTROL_CREATE_BLOCK    0
#define CONTROL_APPEND_BLOCK    1
#define CONTROL_DELETE_BLOCK    2
#define CONTROL_START_BLOCK     3
#define CONTROL_STOP_BLOCK      4
#define CONTROL_RESET           5
#define CONTROL_CREATE_BLOCK_V2 6
#define CONTROL_APPEND_BLOCK_V2 7

#define BLOCK_ID_FREE -1

#define LOG_TYPE_MASK (0x0f)

static const auto TASK_STACK_DEPTH = 2 * configMINIMAL_STACK_SIZE;
StackType_t  logStackBuffer[TASK_STACK_DEPTH]; 
StaticTask_t logTaskBuffer;


typedef enum {
    acqType_memory = 0,
    acqType_function = 1,
} acquisitionType_t;

// Maximum log payload length (4 bytes are used for block id and timestamp)
#define LOG_MAX_LEN 26

/* Log packet parameters storage */
#define LOG_MAX_OPS 128
#define LOG_MAX_BLOCKS 16
struct log_ops {
    struct log_ops * next;
    uint8_t storageType : 4;
    uint8_t logType     : 4;
    void * variable;
    acquisitionType_t acquisitionType;
};

struct log_block {
    int id;
    xTimerHandle timer;
    StaticTimer_t timerBuffer;
    uint32_t droppedPackets;
    struct log_ops * ops;
};

struct ops_setting {
    uint8_t logType;
    uint8_t id;
} __attribute__((packed));

struct ops_setting_v2 {
    uint8_t logType;
    uint16_t id;
} __attribute__((packed));


static bool didInit;

#ifndef TEENSYDUINO

static struct log_ops logOps[LOG_MAX_OPS];
static struct log_block logBlocks[LOG_MAX_BLOCKS];

static xSemaphoreHandle logLock;
static StaticSemaphore_t logLockBuffer;

static struct log_s * logs;
static int logsLen;
static uint32_t logsCrc;
static uint16_t logsCount = 0;

static crtpPacket_t p;


static inline int logGetType(logVarId_t varid)
{
    return logs[varid].type & LOG_TYPE_MASK;
}

static void logTOCProcess(int command)
{
    int ptr = 0;
    char group[10] = {};
    uint16_t n=0;
    uint16_t logId=0;

    strcpy(group, "plop");

    switch (command)
    {
        case CMD_GET_INFO: //Get info packet about the log implementation
            consolePrintf("LOG: Client uses old logging API!\n");
            ptr = 0;
            *group = 0;
            p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
            p.size=8;
            p.data[0]=CMD_GET_INFO;
            if (logsCount < 255) {
                p.data[1]=logsCount;
            } else {
                p.data[1]=255;
            }
            memcpy(&p.data[2], &logsCrc, 4);
            p.data[6]=LOG_MAX_BLOCKS;
            p.data[7]=LOG_MAX_OPS;
            crtpSendPacketBlock(&p);
            break;
        case CMD_GET_ITEM:  //Get log variable
            for (ptr=0; ptr<logsLen; ptr++) //Ptr points a group
            {
                if (logs[ptr].type & LOG_GROUP)
                {
                    if (logs[ptr].type & LOG_START)
                        strcpy(group, logs[ptr].name);
                    else
                        *group = 0;
                }
                else                          //Ptr points a variable
                {
                    if (n==p.data[1])
                        break;
                    n++;
                }
            }

            if (ptr<logsLen)
            {
                p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
                p.data[0]=CMD_GET_ITEM;
                p.data[1]=n;
                p.data[2]=logGetType(ptr);
                p.size=3+2+strlen(group)+strlen(logs[ptr].name);
                memcpy(p.data+3, group, strlen(group)+1);
                memcpy(p.data+3+strlen(group)+1, logs[ptr].name, strlen(logs[ptr].name)+1);
                crtpSendPacketBlock(&p);
            } else {
                p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
                p.data[0]=CMD_GET_ITEM;
                p.size=1;
                crtpSendPacketBlock(&p);
            }
            break;
        case CMD_GET_INFO_V2: //Get info packet about the log implementation
            ptr = 0;
            *group = 0;
            p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
            p.size=9;
            p.data[0]=CMD_GET_INFO_V2;
            memcpy(&p.data[1], &logsCount, 2);
            memcpy(&p.data[3], &logsCrc, 4);
            p.data[7]=LOG_MAX_BLOCKS;
            p.data[8]=LOG_MAX_OPS;
            crtpSendPacketBlock(&p);
            break;
        case CMD_GET_ITEM_V2:  //Get log variable
            memcpy(&logId, &p.data[1], 2);
            for (ptr=0; ptr<logsLen; ptr++) //Ptr points a group
            {
                if (logs[ptr].type & LOG_GROUP)
                {
                    if (logs[ptr].type & LOG_START)
                        strcpy(group, logs[ptr].name);
                    else
                        *group = 0;
                }
                else                          //Ptr points a variable
                {
                    if (n==logId)
                        break;
                    n++;
                }
            }

            if (ptr<logsLen)
            {
                p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
                p.data[0]=CMD_GET_ITEM_V2;
                memcpy(&p.data[1], &logId, 2);
                p.data[3]=logGetType(ptr);
                p.size=4+2+strlen(group)+strlen(logs[ptr].name);
                memcpy(p.data+4, group, strlen(group)+1);
                memcpy(p.data+4+strlen(group)+1, logs[ptr].name, strlen(logs[ptr].name)+1);
                crtpSendPacketBlock(&p);
            } else {
                p.header=CRTP_HEADER(CRTP_PORT_LOG, TOC_CH);
                p.data[0]=CMD_GET_ITEM_V2;
                p.size=1;
                crtpSendPacketBlock(&p);
            }
            break;
    }
}


static void opsFree(struct log_ops * ops)
{
    ops->variable = NULL;
}


static int logDeleteBlock(int id)
{
    int i;
    struct log_ops * ops;
    struct log_ops * opsNext;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == id) break;

    if (i >= LOG_MAX_BLOCKS) {
        return ENOENT;
    }

    ops = logBlocks[i].ops;
    while (ops)
    {
        opsNext = ops->next;
        opsFree(ops);
        ops = opsNext;
    }

    if (logBlocks[i].timer != 0) {
        xTimerStop(logBlocks[i].timer, portMAX_DELAY);
        xTimerDelete(logBlocks[i].timer, portMAX_DELAY);
        logBlocks[i].timer = 0;
    }

    logBlocks[i].id = BLOCK_ID_FREE;
    return 0;
}

/* Appends data to a packet if space is available; returns false on failure. */
static bool appendToPacket(crtpPacket_t * pk, const void * data, size_t n) {
    if (pk->size <= CRTP_MAX_DATA_SIZE - n)
    {
        memcpy(&pk->data[pk->size], data, n);
        pk->size += n;
        return true;
    }
    else return false;
}

static int logStopBlock(int id)
{
    int i;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == id) break;

    if (i >= LOG_MAX_BLOCKS) {
        return ENOENT;
    }

    xTimerStop(logBlocks[i].timer, portMAX_DELAY);

    return 0;
}


static void logReset(void)
{
    int i;

    if (didInit)
    {
        //Stop and delete all started log blocks
        for(i=0; i<LOG_MAX_BLOCKS; i++)
            if (logBlocks[i].id != -1)
            {
                logStopBlock(logBlocks[i].id);
                logDeleteBlock(logBlocks[i].id);
            }
    }

    //Force free all the log block objects
    for(i=0; i<LOG_MAX_BLOCKS; i++)
        logBlocks[i].id = BLOCK_ID_FREE;

    //Force free the log ops
    for (i=0; i<LOG_MAX_OPS; i++)
        logOps[i].variable = NULL;
}



static void logRunBlock(void * arg)
{
    struct log_block *blk = (log_block *)arg;
    struct log_ops *ops = blk->ops;
    static crtpPacket_t pk;
    unsigned int timestamp;

    xSemaphoreTake(logLock, portMAX_DELAY);

    timestamp = ((long long)xTaskGetTickCount())/portTICK_RATE_MS;

    pk.header = CRTP_HEADER(CRTP_PORT_LOG, LOG_CH);
    pk.size = 4;
    pk.data[0] = blk->id;
    pk.data[1] = timestamp&0x0ff;
    pk.data[2] = (timestamp>>8)&0x0ff;
    pk.data[3] = (timestamp>>16)&0x0ff;

    while (ops)
    {
        int valuei = 0;
        float valuef = 0;

        // FPU instructions must run on aligned data.
        // We first copy the data to an (aligned) local variable, before assigning it
        switch(ops->storageType)
        {
            case LOG_UINT8:
                {
                    uint8_t v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->acquireUInt8(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(v));
                    }
                    valuei = v;
                    break;
                }
            case LOG_INT8:
                {
                    int8_t v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->acquireInt8(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(v));
                    }
                    valuei = v;
                    break;
                }
            case LOG_UINT16:
                {
                    uint16_t v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->acquireUInt16(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(v));
                    }
                    valuei = v;
                    break;
                }
            case LOG_INT16:
                {
                    int16_t v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->acquireInt16(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(v));
                    }
                    valuei = v;
                    break;
                }
            case LOG_UINT32:
                {
                    uint32_t v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->acquireUInt32(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(v));
                    }
                    valuei = v;
                    break;
                }
            case LOG_INT32:
                {
                    int32_t v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->acquireInt32(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(v));
                    }
                    valuei = v;
                    break;
                }
            case LOG_FLOAT:
                {
                    float v;
                    if (ops->acquisitionType == acqType_function) {
                        logByFunction_t* logByFunction = (logByFunction_t*)ops->variable;
                        v = logByFunction->aquireFloat(timestamp, logByFunction->data);
                    } else {
                        memcpy(&v, ops->variable, sizeof(valuef));
                    }
                    valuei = v;
                    valuef = v;
                    break;
                }
        }

        if (ops->logType == LOG_FLOAT || ops->logType == LOG_FP16)
        {
            if (ops->storageType != LOG_FLOAT)
            {
                valuef = valuei;
            }

            // Try to append the next item to the packet.  If we run out of space,
            // drop this and subsequent items.
            if (ops->logType == LOG_FLOAT)
            {
                if (!appendToPacket(&pk, &valuef, 4)) break;
            }
            else
            {
                valuei = Num::single2half(valuef);
                if (!appendToPacket(&pk, &valuei, 2)) break;
            }
        }
        else  //logType is an integer
        {
            if (!appendToPacket(&pk, &valuei, typeLengths[ops->logType])) break;
        }

        ops = ops->next;
    }

    xSemaphoreGive(logLock);

    // Check if the connection is still up, oherwise disable
    // all the logging and flush all the CRTP queues.
    if (!crtpIsConnected())
    {
        logReset();
        crtpReset();
    }
    else
    {
        // No need to block here, since logging is not guaranteed
        if (!crtpSendPacket(&pk))
        {
            if (blk->droppedPackets++ % 100 == 0)
            {
                consolePrintf("LOG: WARNING: LOG packets drop detected (%lu packets lost)\n",
                        blk->droppedPackets);
            }
        }
    }
}



static int logStartBlock(int id, unsigned int period)
{
    int i;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == id) break;

    if (i >= LOG_MAX_BLOCKS) {
        return ENOENT;
    }


    if (period>0) {
        xTimerChangePeriod(logBlocks[i].timer, M2T(period), 100);
        xTimerStart(logBlocks[i].timer, 100);
    } 

    // single-shot run
    else {
        extern Worker worker;
        worker.schedule(logRunBlock, &logBlocks[i]);
    }

    return 0;
}

static int blockCalcLength(struct log_block * block)
{
    struct log_ops * ops;
    int len = 0;

    for (ops = block->ops; ops; ops = ops->next)
        len += typeLengths[ops->logType];

    return len;
}

static struct log_ops * opsMalloc()
{
    int i;

    for (i=0;i<LOG_MAX_OPS; i++)
        if (logOps[i].variable == NULL) break;

    if (i >= LOG_MAX_OPS)
        return NULL;

    return &logOps[i];
}


static int variableGetIndex(int id)
{
    int i;
    int n=0;

    for (i=0; i<logsLen; i++)
    {
        if(!(logs[i].type & LOG_GROUP))
        {
            if(n==id)
                break;
            n++;
        }
    }

    if (i>=logsLen)
        return -1;

    return i;
}

static acquisitionType_t acquisitionTypeFromLogType(uint8_t logType) {
    if (logType & LOG_BY_FUNCTION) {
        return acqType_function;
    }

    return acqType_memory;
}

static void blockAppendOps(struct log_block * block, struct log_ops * ops)
{
    struct log_ops * o;

    ops->next = NULL;

    if (block->ops == NULL)
        block->ops = ops;
    else
    {
        for (o = block->ops; o->next; o = o->next);

        o->next = ops;
    }
}



static int logAppendBlock(int id, struct ops_setting * settings, int len)
{
    int i;
    struct log_block * block;


    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == id) break;

    if (i >= LOG_MAX_BLOCKS) {
        return ENOENT;
    }

    block = &logBlocks[i];

    for (i=0; i<len; i++)
    {
        int currentLength = blockCalcLength(block);
        struct log_ops * ops;
        int varId;

        if ((currentLength + typeLengths[settings[i].logType & LOG_TYPE_MASK])>LOG_MAX_LEN) {
            return E2BIG;
        }

        ops = opsMalloc();

        if(!ops) {
            return ENOMEM;
        }

        if (settings[i].id != 255)  //TOC variable
        {
            varId = variableGetIndex(settings[i].id);

            if (varId<0) {
                return ENOENT;
            }

            ops->variable    = logs[varId].address;
            ops->storageType = logGetType(varId);
            ops->logType     = settings[i].logType & LOG_TYPE_MASK;
            ops->acquisitionType = acquisitionTypeFromLogType(logs[varId].type);

        } else {                     //Memory variable
            //TODO: Check that the address is in ram
            ops->variable    = (void*)(&settings[i]+1);
            ops->storageType = (settings[i].logType>>4) & LOG_TYPE_MASK;
            ops->logType     = settings[i].logType & LOG_TYPE_MASK;
            ops->acquisitionType = acqType_memory;
            i += 2;

        }
        blockAppendOps(block, ops);

    }

    return 0;
}

static int logAppendBlockV2(int id, struct ops_setting_v2 * settings, int len)
{
    int i;
    struct log_block * block;


    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == id) break;

    if (i >= LOG_MAX_BLOCKS) {
        return ENOENT;
    }

    block = &logBlocks[i];

    for (i=0; i<len; i++)
    {
        int currentLength = blockCalcLength(block);
        struct log_ops * ops;
        int varId;

        if ((currentLength + typeLengths[settings[i].logType & LOG_TYPE_MASK])>LOG_MAX_LEN) {
            return E2BIG;
        }

        ops = opsMalloc();

        if(!ops) {
            return ENOMEM;
        }

        if (settings[i].id != 0xFFFFul)  //TOC variable
        {
            varId = variableGetIndex(settings[i].id);

            if (varId<0) {
                return ENOENT;
            }

            ops->variable    = logs[varId].address;
            ops->storageType = logGetType(varId);
            ops->logType     = settings[i].logType & LOG_TYPE_MASK;
            ops->acquisitionType = acquisitionTypeFromLogType(logs[varId].type);

        } else {                     //Memory variable
            //TODO: Check that the address is in ram
            ops->variable    = (void*)(&settings[i]+1);
            ops->storageType = (settings[i].logType>>4) & LOG_TYPE_MASK;
            ops->logType     = settings[i].logType & LOG_TYPE_MASK;
            ops->acquisitionType = acqType_memory;
            i += 2;

        }
        blockAppendOps(block, ops);

    }

    return 0;
}


static void logBlockTimed(xTimerHandle timer)
{
    extern Worker worker;
    worker.schedule(logRunBlock, pvTimerGetTimerID(timer));
}


static int logCreateBlockV2(unsigned char id, struct ops_setting_v2 * settings, int len)
{
    int i;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (id == logBlocks[i].id) return EEXIST;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == BLOCK_ID_FREE) break;

    if (i == LOG_MAX_BLOCKS)
        return ENOMEM;

    logBlocks[i].id = id;
    logBlocks[i].timer = xTimerCreateStatic("logTimer", M2T(1000), pdTRUE,
            &logBlocks[i], logBlockTimed, &logBlocks[i].timerBuffer);
    logBlocks[i].ops = NULL;

    if (logBlocks[i].timer == NULL)
    {
        logBlocks[i].id = BLOCK_ID_FREE;
        return ENOMEM;
    }


    return logAppendBlockV2(id, settings, len);
}


static int logCreateBlock(unsigned char id, struct ops_setting * settings, int len)
{
    int i;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (id == logBlocks[i].id) return EEXIST;

    for (i=0; i<LOG_MAX_BLOCKS; i++)
        if (logBlocks[i].id == BLOCK_ID_FREE) break;

    if (i == LOG_MAX_BLOCKS)
        return ENOMEM;

    logBlocks[i].id = id;
    logBlocks[i].timer = xTimerCreateStatic("logTimer", M2T(1000), pdTRUE,
            &logBlocks[i], logBlockTimed, &logBlocks[i].timerBuffer);
    logBlocks[i].ops = NULL;

    if (logBlocks[i].timer == NULL)
    {
        logBlocks[i].id = BLOCK_ID_FREE;
        return ENOMEM;
    }


    return logAppendBlock(id, settings, len);
}



static void logControlProcess()
{
    int ret = ENOEXEC;

    switch(p.data[0])
    {
        case CONTROL_CREATE_BLOCK:
            ret = logCreateBlock( p.data[1],
                    (struct ops_setting*)&p.data[2],
                    (p.size-2)/sizeof(struct ops_setting) );
            break;
        case CONTROL_APPEND_BLOCK:
            ret = logAppendBlock( p.data[1],
                    (struct ops_setting*)&p.data[2],
                    (p.size-2)/sizeof(struct ops_setting) );
            break;
        case CONTROL_DELETE_BLOCK:
            ret = logDeleteBlock( p.data[1] );
            break;
        case CONTROL_START_BLOCK:
            ret = logStartBlock( p.data[1], p.data[2]*10);
            break;
        case CONTROL_STOP_BLOCK:
            ret = logStopBlock( p.data[1] );
            break;
        case CONTROL_RESET:
            logReset();
            ret = 0;
            break;
        case CONTROL_CREATE_BLOCK_V2:
            ret = logCreateBlockV2( p.data[1],
                    (struct ops_setting_v2*)&p.data[2],
                    (p.size-2)/sizeof(struct ops_setting_v2) );
            break;
        case CONTROL_APPEND_BLOCK_V2:
            ret = logAppendBlockV2( p.data[1],
                    (struct ops_setting_v2*)&p.data[2],
                    (p.size-2)/sizeof(struct ops_setting_v2) );
            break;
    }

    //Commands answer
    p.data[2] = ret;
    p.size = 3;
    crtpSendPacketBlock(&p);
}


static void logTask(void * prm)
{
    crtpInitTaskQueue(CRTP_PORT_LOG);

    while(1) {
        crtpReceivePacketBlock(CRTP_PORT_LOG, &p);

        xSemaphoreTake(logLock, portMAX_DELAY);
        if (p.channel==TOC_CH)
            logTOCProcess(p.data[0]);
        if (p.channel==CONTROL_CH)
            logControlProcess();
        xSemaphoreGive(logLock);
    }
}

#endif // not TEENSYDUINO

/////////////////////////////////////////////////////////////////////////

void logInit(void)
{
#ifndef TEENSYDUINO
    //These are set by the Linker
    extern struct log_s _log_start;
    extern struct log_s _log_stop;

    const char* group = NULL;
    int groupLength = 0;

    if(didInit) {
        return;
    }

    logs = &_log_start;
    logsLen = &_log_stop - &_log_start;

    // Calculate a hash of the toc by chaining description of each elements
    // Using the CRTP packet as temporary buffer
    logsCrc = 0;
    for (uint8_t i=0; i<logsLen; i++) {
        int len = 5;
        memcpy(&p.data[0], &logsCrc, 4);
        p.data[4] = logs[i].type;
        if (logs[i].type & LOG_GROUP) {
            if (logs[i].type & LOG_START) {
                group = logs[i].name;
                groupLength = strlen(group);
            }
        } else {
            // CMD_GET_ITEM_V2 result's size is: 3 + strlen(logs[i].name) + groupLength + 2
            if (strlen(logs[i].name) + groupLength + 2 > 26) {
            }
        }
        if (logs[i].name) {
            memcpy(&p.data[5], logs[i].name, strlen(logs[i].name));
            len += strlen(logs[i].name);
        }
        logsCrc = Crc32::calculateBuffer(p.data, len);
    }

    // Big lock that protects the log datastructures
    logLock = xSemaphoreCreateMutexStatic(&logLockBuffer);

    for (uint8_t i=0; i<logsLen; i++) {
        if(!(logs[i].type & LOG_GROUP))
            logsCount++;
    }

    //Manually free all log blocks
    for(uint8_t i=0; i<LOG_MAX_BLOCKS; i++)
        logBlocks[i].id = BLOCK_ID_FREE;

    //Init data structures and set the log subsystem in a known state
    logReset();

    //Start the log task
    xTaskCreateStatic(
            logTask, 
            "LOG", 
            2 * configMINIMAL_STACK_SIZE, 
            NULL, 
            1, 
            logStackBuffer,
            &logTaskBuffer);
#endif

    didInit = true;
}

bool logTest(void)
{
    return didInit;
}

static float unused;


//////////////////////////////////////////////////////////////////////////////

extern Safety safety;
extern CoreTask coreTask;
extern bool didResetEstimation;

extern RadioLink radioLink;
extern uint32_t motor_ratios[];
extern uint32_t memTesterWriteErrorCount;
extern PowerMonitorTask powerMonitorTask;
extern PowerMonitorTask::syslinkInfo_t pmSyslinkInfo;
extern crtpStats_t crtpStats;

//////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(radio)
    LOG_ADD_CORE(LOG_UINT8, rssi, &radioLink.rssi)
    LOG_ADD_CORE(LOG_UINT8, isConnected, &radioLink.isConnectedFlag)
LOG_GROUP_STOP(radio)

    //////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(motor)
    LOG_ADD_CORE(LOG_UINT32, m1, &motor_ratios[0])
    LOG_ADD_CORE(LOG_UINT32, m2, &motor_ratios[1])
    LOG_ADD_CORE(LOG_UINT32, m3, &motor_ratios[2])
    LOG_ADD_CORE(LOG_UINT32, m4, &motor_ratios[3])
LOG_GROUP_STOP(motor)


    //////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(sys)
    LOG_ADD_CORE(LOG_UINT8, canfly, &safety.canFlyFlag)
    LOG_ADD_CORE(LOG_UINT8, isFlying, &safety.isFlyingFlag)
    LOG_ADD_CORE(LOG_UINT8, isTumbled, &safety.isTumbledFlag)
LOG_GROUP_STOP(sys)

    LOG_GROUP_START(supervisor)
    LOG_ADD(LOG_UINT16, info, &safety.infoBitfield)
LOG_GROUP_STOP(supervisor)


    //////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(kalman)
    LOG_ADD(LOG_FLOAT, stateX, &unused)
    LOG_ADD(LOG_FLOAT, stateY, &unused)
    LOG_ADD(LOG_FLOAT, stateZ, &unused)
    LOG_ADD(LOG_FLOAT, statePX, &unused)
    LOG_ADD(LOG_FLOAT, statePY, &unused)
    LOG_ADD(LOG_FLOAT, statePZ, &unused)
    LOG_ADD(LOG_FLOAT, stateD0, &unused)
    LOG_ADD(LOG_FLOAT, stateD1, &unused)
    LOG_ADD(LOG_FLOAT, stateD2, &unused)
    LOG_ADD(LOG_FLOAT, varX, &unused)
    LOG_ADD(LOG_FLOAT, varY, &unused)
    LOG_ADD(LOG_FLOAT, varZ, &unused)
    LOG_ADD(LOG_FLOAT, varPX, &unused)
    LOG_ADD(LOG_FLOAT, varPY, &unused)
    LOG_ADD(LOG_FLOAT, varPZ, &unused)
    LOG_ADD(LOG_FLOAT, varD0, &unused)
    LOG_ADD(LOG_FLOAT, varD1, &unused)
    LOG_ADD(LOG_FLOAT, varD2, &unused)
    LOG_ADD(LOG_FLOAT, q0, &unused)
    LOG_ADD(LOG_FLOAT, q1, &unused)
    LOG_ADD(LOG_FLOAT, q2, &unused)
    LOG_ADD(LOG_FLOAT, q3, &unused)
LOG_GROUP_STOP(kalman)

    LOG_GROUP_START(kalman_pred)
    LOG_ADD(LOG_FLOAT, predNX, &unused)
    LOG_ADD(LOG_FLOAT, predNY, &unused)
    LOG_ADD(LOG_FLOAT, measNX, &unused)
    LOG_ADD(LOG_FLOAT, measNY, &unused)
LOG_GROUP_STOP(kalman_pred)

    LOG_GROUP_START(stabilizer)
    LOG_ADD(LOG_FLOAT, thrust, &unused)
    LOG_ADD(LOG_FLOAT, roll, &unused)
    LOG_ADD(LOG_FLOAT, pitch, &unused)
    LOG_ADD(LOG_FLOAT, yaw, &unused)
LOG_GROUP_STOP(stabilizer)

    LOG_GROUP_START(controller)
    LOG_ADD(LOG_INT16, ctr_yaw, &unused)
LOG_GROUP_STOP(controller)

    LOG_GROUP_START(stateEstimate)
    LOG_ADD_CORE(LOG_FLOAT, x, &coreTask.vehicleState.x)
    LOG_ADD_CORE(LOG_FLOAT, y, &coreTask.vehicleState.y)
    LOG_ADD_CORE(LOG_FLOAT, z, &coreTask.vehicleState.z)
    LOG_ADD_CORE(LOG_FLOAT, vx, &coreTask.vehicleState.dx)
    LOG_ADD_CORE(LOG_FLOAT, vy, &coreTask.vehicleState.dy)
    LOG_ADD_CORE(LOG_FLOAT, vz, &coreTask.vehicleState.dz)
    LOG_ADD_CORE(LOG_FLOAT, roll, &coreTask.vehicleState.phi)
    LOG_ADD_CORE(LOG_FLOAT, pitch, &coreTask.vehicleState.theta)
    LOG_ADD_CORE(LOG_FLOAT, yaw, &coreTask.vehicleState.psi)
LOG_GROUP_STOP(stateEstimate)

    ///////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(memTst)
    LOG_ADD(LOG_UINT32, errCntW, &memTesterWriteErrorCount)
LOG_GROUP_STOP(memTst)

    ///////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(pm)
    LOG_ADD_CORE(LOG_FLOAT, vbat, &powerMonitorTask.batteryVoltage)
    LOG_ADD(LOG_UINT16, vbatMV, &powerMonitorTask.batteryVoltageMV)
    LOG_ADD(LOG_FLOAT, extVbat, &powerMonitorTask.extBatteryVoltage)
    LOG_ADD(LOG_UINT16, extVbatMV, &powerMonitorTask.extBatteryVoltageMV)
    LOG_ADD(LOG_FLOAT, extCurr, &powerMonitorTask.extBatteryCurrent)
    LOG_ADD(LOG_FLOAT, chargeCurrent, &pmSyslinkInfo.chargeCurrent)
    LOG_ADD_CORE(LOG_INT8, state, &powerMonitorTask.state)
LOG_ADD_CORE(LOG_UINT8, batteryLevel, &powerMonitorTask.batteryLevel)
#ifdef PM_SYSTLINK_INCLUDE_TEMP
LOG_ADD(LOG_FLOAT, temp, &temp)
#endif
LOG_GROUP_STOP(pm)

    ///////////////////////////////////////////////////////////////////////////////

    LOG_GROUP_START(crtp)
    LOG_ADD(LOG_UINT16, rxRate, &crtpStats.rxRate)
    LOG_ADD(LOG_UINT16, txRate, &crtpStats.txRate)
LOG_GROUP_STOP(crtp)

