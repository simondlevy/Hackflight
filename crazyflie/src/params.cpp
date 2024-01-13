/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2023 Bitcraze AB
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
 * param.logic.c - Crazy parameter system logic source file.
 */
#include <string.h>
#include <errno.h>

#include <free_rtos.h>

#include <crc32.hpp>

#include <crtp/crtp.h>

#include <tasks/estimator.hpp>
#include <tasks/core.hpp>
#include <tasks/flowdeck.hpp>
#include <tasks/zranger.hpp>

#include <tasks/power.hpp>

#include <console.h>
#include <config.h>
#include <params.h>
#include <type_lengths.h>
#include <storage.h>
#include <param_macros.h>
#include <safety.hpp>

#define CMD_RESET 0
#define CMD_GET_NEXT 1
#define CMD_GET_CRC 2

#define CMD_GET_ITEM    0 // original version: up to 255 entries
#define CMD_GET_INFO    1 // original version: up to 255 entries
#define CMD_GET_ITEM_V2 2 // version 2: up to 16k entries
#define CMD_GET_INFO_V2 3 // version 2: up to 16k entries

#define PERSISTENT_PREFIX_STRING "prm/"

/** Check variable ID validity
 *
 * @param varId variable ID, returned by paramGetVarId()
 * @return true if the variable ID is valid, false otherwise.
 */
#define PARAM_VARID_IS_VALID(varId) (varId.id != 0xffffu)

#define KEY_LEN 30  // FIXME

typedef struct paramVarId_s {
    uint16_t id;
    uint16_t index;
} __attribute__((packed)) paramVarId_t;

static paramVarId_t invalidVarId = {0xffffu, 0xffffu};

//Private functions
static int variableGetIndex(int id);
static void paramNotifyChanged(int index);
static char paramWriteByNameProcess(char* group, char* name, int type, void *valptr);


//Pointer to the parameters list and length of it
static struct param_s * params;
static int paramsLen;
static uint32_t paramsCrc;
static uint16_t paramsCount = 0;

// _sdata is from linker script and points to start of data section
extern int _sdata;
extern int _edata;
// sidata is from linker script and points to start of initialization data flash section
extern int _sidata;
// _stext is from linker script and points to start of flash section
extern int _stext;
extern int _etext;
static const uint64_t dummyZero64 = 0;

static void * paramGetDefault(int index)
{
    uint32_t valueRelative;
    uint32_t address;
    void *ptrDefaultValue;

    address = (uint32_t)(params[index].address);

    // Is variable in data section?
    if (address >= (uint32_t)&_sdata &&
            address <= (uint32_t)&_edata)
    {
        valueRelative =  address - (uint32_t)&_sdata;
        ptrDefaultValue = (void *)((uint32_t)&_sidata + valueRelative);
    }
    // Is variable in flash section?
    else if (address >= (uint32_t)&_stext &&
            address <= (uint32_t)&_etext)
    {
        ptrDefaultValue = (void *)(address);
    }
    // It is zero
    else
    {
        ptrDefaultValue = (void *)&dummyZero64;
    }

    return ptrDefaultValue;
}

/**
 * Set param with [index] to data
 *
 * @param index  The param index
 * @param data  The variable data
 *
 * @return number of bytes set
 **/
static int paramSet(uint16_t index, void *data)
{
    int paramLength = 0;

    switch (params[index].type & PARAM_BYTES_MASK)
    {
        case PARAM_1BYTE:
            paramLength = 1;
            break;
        case PARAM_2BYTES:
            paramLength = 2;
            break;
        case PARAM_4BYTES:
            paramLength = 4;
            break;
        case PARAM_8BYTES:
            paramLength = 8;
            break;
    }

    memcpy(params[index].address, data, paramLength);

    return paramLength;
}

/**
 * Get param with [index]
 *
 * @param index  The param index
 * @param data  The variable data
 *
 * @return number of bytes read
 **/
static int paramGet(uint16_t index, void *data)
{
    int paramLength = 0;

    switch (params[index].type & PARAM_BYTES_MASK)
    {
        case PARAM_1BYTE:
            paramLength = 1;
            break;
        case PARAM_2BYTES:
            paramLength = 2;
            break;
        case PARAM_4BYTES:
            paramLength = 4;
            break;
        case PARAM_8BYTES:
            paramLength = 8;
            break;
    }

    memcpy(data, params[index].address, paramLength);

    return paramLength;
}

/**
 * Get param on [index] length in bytes
 *
 * @return number of bytes
 **/
static int paramGetLen(uint16_t index)
{
    int paramLength = 0;

    switch (params[index].type & PARAM_BYTES_MASK)
    {
        case PARAM_1BYTE:
            paramLength = 1;
            break;
        case PARAM_2BYTES:
            paramLength = 2;
            break;
        case PARAM_4BYTES:
            paramLength = 4;
            break;
        case PARAM_8BYTES:
            paramLength = 8;
            break;
    }

    return paramLength;
}



static void paramNotifyChanged(int index) {
    if (params[index].callback) {
        params[index].callback();
    }
}

static char paramWriteByNameProcess(char* group, char* name, int type, void *valptr) {
    int index;
    char pgroup[100] = {};

    for (index = 0; index < paramsLen; index++) //Ptr points a group
    {
        if (params[index].type & PARAM_GROUP)
        {
            if (params[index].type & PARAM_START) {
                strcpy(pgroup, params[index].name);
            }
            else {
                *pgroup = 0;
            }
        }
        else                          //Ptr points a variable
        {
            if (!strcmp(params[index].name, name) && !strcmp(pgroup, group))
                break;
        }
    }

    if (index >= paramsLen) {
        return ENOENT;
    }

    if (type != (params[index].type & (~(PARAM_CORE | PARAM_RONLY | PARAM_EXTENDED)))) {
        return EINVAL;
    }

    if (params[index].type & PARAM_RONLY) {
        return EACCES;
    }

    paramSet(index, valptr);

    paramNotifyChanged(index);

    return 0;
}

static paramVarId_t paramGetVarId(const char* group, const char* name)
{
    uint16_t index;
    uint16_t id = 0;
    paramVarId_t varId = invalidVarId;
    char currgroup[100] = {};

    for(index = 0; index < paramsLen; index++)
    {
        if (params[index].type & PARAM_GROUP) {
            if (params[index].type & PARAM_START) {
                strcpy(currgroup, params[index].name);
            }
        } else {
            id += 1;
        }

        if ((!strcmp(group, currgroup)) && (!strcmp(name, params[index].name))) {
            varId.index = index;
            varId.id = id - 1;
            return varId;
        }
    }

    return invalidVarId;
}

static int variableGetIndex(int id)
{
    int i;
    int n = 0;

    for (i = 0; i < paramsLen; i++)
    {
        if(!(params[i].type & PARAM_GROUP))
        {
            if(n == id) {
                break;
            }
            n++;
        }
    }

    if (i >= paramsLen)
        return -1;

    return i;
}

static paramVarId_t paramGetVarIdFromComplete(const char* completeName)
{
    char group[32] = { 0, };

    char *dot = strchr(completeName, '.');
    if (!dot) {
        return invalidVarId;
    }

    size_t group_len = dot - completeName;
    memcpy(group, completeName, group_len);
    char *name = (char *) (dot + 1);

    return paramGetVarId(group, name);
}



static void paramGetGroupAndName(paramVarId_t varid, char * group, char * name)
{
    char currgroup[100] = {};
    *group = 0;
    *name = 0;

    for(int index = 0; index < paramsLen; index++) {
        if (params[index].type & PARAM_GROUP) {
            if (params[index].type & PARAM_START) {
                strcpy(currgroup, params[index].name);
            }
        }

        if (index == varid.index) {
            strcpy(group, currgroup);
            strcpy(name, params[index].name);
            break;
        }
    }
}

static void generateStorageKey(const uint16_t index, char key[KEY_LEN])
{
    char group[100] = {};
    char name[100] = {};
    paramVarId_t paramId;

    paramId.index = (uint16_t)index;
    paramGetGroupAndName(paramId, group, name);

    // Assemble key string, e.g. "prm/pid_rate.kp"
    strcpy(key, PERSISTENT_PREFIX_STRING);
    strcat(key, group);
    strcat(key, ".");
    strcat(key, name);
}

static bool persistentParamFromStorage(const char *key, void *buffer, size_t length)
{
    //
    // The key is of format "prm/group.name", we need group and name.
    //
    char *completeName = (char *) key + strlen(PERSISTENT_PREFIX_STRING);
    paramVarId_t varId = paramGetVarIdFromComplete(completeName);

    if (PARAM_VARID_IS_VALID(varId)) {
        paramSet(varId.index, buffer);
    }

    return true;
}

static void paramGetDefaultValue(crtpPacket_t *p)
{
    uint16_t id;

    memcpy(&id, &p->data[1], sizeof(id));
    int index = variableGetIndex(id);

    const bool doesParamExist = (index >= 0);
    // Read-only parameters have no default value
    if (!doesParamExist || params[index].type & PARAM_RONLY) {
        p->data[3] = ENOENT;
        p->size = 4;
        crtpSendPacketBlock(p);
        return;
    }

    // Add default value
    uint8_t paramLen = paramGetLen(index);
    if (params[index].getter) {
        memcpy(&p->data[3], params[index].getter(), paramLen);
    } else {
        memcpy(&p->data[3], paramGetDefault(index), paramLen);
    }
    p->size = 3 + paramLen;
    crtpSendPacketBlock(p);
}


static void paramLogicStorageInit()
{
    storageForeach(PERSISTENT_PREFIX_STRING, persistentParamFromStorage);
}

static void paramPersistentClear(crtpPacket_t *p)
{
    int index;
    uint16_t id;
    bool result = true;

    memcpy(&id, &p->data[1], 2);
    index = variableGetIndex(id);

    if (index < 0) {
        p->data[3] = ENOENT;
        p->size = 4;
        crtpSendPacketBlock(p);
        return;
    }

    // Assemble key string, e.g. "prm/pid_rate.kp"
    char key[KEY_LEN] = {0};
    generateStorageKey(index, key);

    result = storageDelete(key);

    p->data[3] = result ? 0: ENOENT;
    p->size = 4;
    crtpSendPacketBlock(p);
}

static void paramPersistentGetState(crtpPacket_t *p)
{
    uint16_t id;

    memcpy(&id, &p->data[1], 2);
    int index = variableGetIndex(id);

    const bool doesParamExist = (index >= 0);
    if (! doesParamExist) {
        p->data[3] = ENOENT;
        p->size = 4;
        crtpSendPacketBlock(p);
        return;
    }

    char key[KEY_LEN] = {0};
    generateStorageKey(index, key);

    uint8_t paramLen = paramGetLen(index);

    // First part of data use 4 bytes
    p->size = 4;

    // Add default value
    if (params[index].getter) {
        memcpy(&p->data[p->size], params[index].getter(), paramLen);
    } else {
        memcpy(&p->data[p->size], paramGetDefault(index), paramLen);
    }
    p->size += paramLen;

    // Add stored value if avialable
    uint8_t value[8];
    const bool isValueStored = (storageFetch(key, &value, paramLen) > 0);
    if (isValueStored) {
        p->data[3] = PARAM_PERSISTENT_STORED;
        memcpy(&p->data[p->size], &value, paramLen);
        p->size += paramLen;
    } else {
        p->data[3] = PARAM_PERSISTENT_NOT_STORED;
    }

    crtpSendPacketBlock(p);
}

static void paramPersistentStore(crtpPacket_t *p)
{
    int index;
    uint16_t id;
    bool result = true;

    memcpy(&id, &p->data[1], 2);
    index = variableGetIndex(id);

    if (index < 0) {
        p->data[3] = ENOENT;
        p->size = 4;
        crtpSendPacketBlock(p);
        return;
    }

    char key[KEY_LEN] = {0};
    generateStorageKey(index, key);

    result = storageStore(key, params[index].address, paramGetLen(index));

    p->data[3] = result ? 0: ENOENT;
    p->size = 4;
    crtpSendPacketBlock(p);
}

static void paramGetExtendedType(crtpPacket_t *p)
{
    int index;
    uint16_t id;

    memcpy(&id, &p->data[1], 2);
    index = variableGetIndex(id);

    if (index < 0 || !(params[index].type & PARAM_EXTENDED)) {
        p->data[3] = ENOENT;
        p->size = 4;
        crtpSendPacketBlock(p);
        return;
    }

    p->data[3] = params[index].extended_type;
    p->size = 4;

    crtpSendPacketBlock(p);
}

static void paramLogicInit(void)
{
#ifndef TEENSYDUINO
    extern struct param_s _param_start;
    extern struct param_s _param_stop;

    const char* group = NULL;
    int groupLength = 0;
    uint8_t buf[30];

    params = &_param_start;
    paramsLen = &_param_stop - &_param_start;

    // Calculate a hash of the toc by chaining description of each elements
    paramsCrc = 0;
    for (uint8_t i=0; i<paramsLen; i++) {
        int len = 5;
        memcpy(&buf[0], &paramsCrc, 4);
        buf[4] = params[i].type;
        if (params[i].type & PARAM_GROUP) {
            if (params[i].type & PARAM_START) {
                group = params[i].name;
                groupLength = strlen(group);
            }
        } else {
            // CMD_GET_ITEM_V2 result's size is: 4 + strlen(params[i].name) + groupLength + 2
            if (strlen(params[i].name) + groupLength + 2 > 26) {
            }
        }

        if (params[i].name) {
            memcpy(&buf[5], params[i].name, strlen(params[i].name));
            len += strlen(params[i].name);
        }
        paramsCrc = Crc32::calculateBuffer(buf, len);
    }

    for (uint8_t i=0; i<paramsLen; i++)
    {
        if(!(params[i].type & PARAM_GROUP))
            paramsCount++;
    }
#endif
}

static void paramTOCProcess(crtpPacket_t *p, int command)
{
    int ptr = 0;
    char group[100] = {};
    uint16_t n=0;
    uint16_t paramId=0;

    switch (command)
    {
        case CMD_GET_INFO: //Get info packet about the param implementation (obsolete)
            consolePrintf("PARAMS: Param API V1 not supported anymore!\n");
            ptr = 0;
            *group = 0;
            p->header = CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
            p->size = 4;
            p->data[0] = CMD_GET_INFO;
            p->data[1] = 0; // Param count
            crtpSendPacketBlock(p);
            break;
        case CMD_GET_ITEM:  //Get param variable (obsolete)
            consolePrintf("PARAMS: Param API V1 not supported anymore!\n");
            p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
            p->data[0]=CMD_GET_ITEM;
            p->size=1;
            crtpSendPacketBlock(p);
            break;
        case CMD_GET_INFO_V2: //Get info packet about the param implementation
            ptr = 0;
            *group = 0;
            p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
            p->size=7;
            p->data[0]=CMD_GET_INFO_V2;
            memcpy(&p->data[1], &paramsCount, 2);
            memcpy(&p->data[3], &paramsCrc, 4);
            crtpSendPacketBlock(p);
            break;
        case CMD_GET_ITEM_V2:  //Get param variable
            memcpy(&paramId, &p->data[1], 2);
            for (ptr=0; ptr<paramsLen; ptr++) //Ptr points a group
            {
                if (params[ptr].type & PARAM_GROUP)
                {
                    if (params[ptr].type & PARAM_START)
                        strcpy(group, params[ptr].name);
                    else
                        *group = 0;
                }
                else                          //Ptr points a variable
                {
                    if (n==paramId)
                        break;
                    n++;
                }
            }

            if (ptr<paramsLen)
            {
                p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
                p->data[0]=CMD_GET_ITEM_V2;
                memcpy(&p->data[1], &paramId, 2);
                p->data[3] = params[ptr].type;
                p->size = 4 + 2 + strlen(group) + strlen(params[ptr].name);
                memcpy(p->data+4, group, strlen(group)+1);
                memcpy(p->data+4+strlen(group)+1, params[ptr].name, strlen(params[ptr].name)+1);
                crtpSendPacketBlock(p);
            } else {
                p->header=CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
                p->data[0]=CMD_GET_ITEM_V2;
                p->size=1;
                crtpSendPacketBlock(p);
            }
            break;
    }
}

static void paramWriteProcess(crtpPacket_t *p)
{
    uint16_t id;
    memcpy(&id, &p->data[0], 2);

    void* valptr = &p->data[2];
    int index;

    index = variableGetIndex(id);

    if (index < 0) {
        p->data[2] = ENOENT;
        p->size = 3;

        crtpSendPacketBlock(p);
        return;
    }

    if (params[index].type & PARAM_RONLY)
        return;

    paramSet(index, valptr);

    crtpSendPacketBlock(p);

    paramNotifyChanged(index);
}

static void paramReadProcess(crtpPacket_t *p)
{
    uint16_t id;
    memcpy(&id, &p->data[0], 2);
    int index = variableGetIndex(id);

    if (index<0) {
        p->data[2] = ENOENT;
        p->size = 3;

        crtpSendPacketBlock(p);
        return;
    }
    p->data[2] = 0;
    p->size = 3 + paramGet(index, &p->data[3]);

    crtpSendPacketBlock(p);
}

static void paramSetByName(crtpPacket_t *p)
{
    int i, nzero = 0;
    char *group;
    char *name;
    uint8_t type;
    void * valPtr;
    int error;

    // If the packet contains at least 2 zeros in the first 28 bytes
    // The packet decoding algorithm will not crash
    for (i = 0; i < CRTP_MAX_DATA_SIZE; i++) {
        if (p->data[i] == '\0') nzero++;
    }

    if (nzero < 2) return;

    group = (char*)&p->data[1];
    name = (char*)&p->data[1 + strlen(group) + 1];
    type = p->data[1 + strlen(group) + 1 + strlen(name) + 1];
    valPtr = &p->data[1 + strlen(group) + 1 + strlen(name) + 2];

    error = paramWriteByNameProcess(group, name, type, valPtr);  /* calls callback */

    p->data[1 + strlen(group) + 1 + strlen(name) + 1] = error;
    p->size = 1 + strlen(group) + 1 + strlen(name) + 1 + 1;
    crtpSendPacketBlock(p);
}

//////////////////////////////////////////////////////////////////////////////

void runParamTask(void)
{
    paramLogicInit();
    paramLogicStorageInit();

    crtpInitTaskQueue(CRTP_PORT_PARAM);

    while (true) {

        crtpPacket_t p = {};

        crtpReceivePacketBlock(CRTP_PORT_PARAM, &p);

        if (p.channel==TOC_CH)
            paramTOCProcess(&p, p.data[0]);
        else if (p.channel==READ_CH)
            paramReadProcess(&p);
        else if (p.channel==WRITE_CH)
            paramWriteProcess(&p);
        else if (p.channel==MISC_CH) {
            switch (p.data[0]) {
                case MISC_SETBYNAME:
                    paramSetByName(&p);
                    break;
                case MISC_GET_EXTENDED_TYPE:
                    paramGetExtendedType(&p);
                    break;
                case MISC_PERSISTENT_STORE:
                    paramPersistentStore(&p);
                    break;
                case MISC_PERSISTENT_GET_STATE:
                    paramPersistentGetState(&p);
                    break;
                case MISC_PERSISTENT_CLEAR:
                    paramPersistentClear(&p);
                    break;
                case MISC_GET_DEFAULT_VALUE:
                    paramGetDefaultValue(&p);
                    break;
                default:
                    break;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////

extern CoreTask coreTask;
extern EstimatorTask estimatorTask;
extern FlowDeckTask flowDeckTask;
extern ZRangerTask zrangerTask;
extern Safety safety;
extern uint8_t sysload_triggerDump;
extern bool storageStats;
extern bool reformatValue;
extern CoreTask coreTask;
extern bool motorSetEnable;
extern uint16_t motorPowerSet[]; 
extern uint8_t memTesterWriteReset;
extern PowerMonitorTask powerMonitorTask;
extern uint8_t syslink_triggerDebugProbe;
extern uint16_t system_echoDelay;

//////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(stabilizer)
    PARAM_ADD_CORE(PARAM_UINT8, stop, &safety.paramEmergencyStop)
    PARAM_ADD_CORE(PARAM_UINT8, taskDump, &sysload_triggerDump)
PARAM_GROUP_STOP(stabilizer)

    //////////////////////////////////////////////////////////////////////////////

static void printStats(void)
{
    if (storageStats) {
        storagePrintStats();
        storageStats = false;
    }
}


static void doReformat(void)
{
    if (reformatValue) {
        storageReformat();
    }
}

//////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(system)
    PARAM_ADD_CORE(PARAM_INT8, arm, &safety.deprecatedArmParam)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, storageStats, &storageStats, printStats)
    PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, storageReformat, &reformatValue, doReformat)
PARAM_GROUP_STOP(system)

    PARAM_GROUP_START(supervisor)
    PARAM_ADD(PARAM_UINT8, infdmp, &safety.doinfodump)
PARAM_GROUP_STOP(supervisor)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(kalman)
    PARAM_ADD_CORE(PARAM_UINT8, resetEstimation, &estimatorTask.didResetEstimation)
PARAM_GROUP_STOP(kalman)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(motorPowerSet)
    PARAM_ADD_CORE(PARAM_UINT8, enable, &motorSetEnable)
    PARAM_ADD_CORE(PARAM_UINT16, m1, &motorPowerSet[0])
    PARAM_ADD_CORE(PARAM_UINT16, m2, &motorPowerSet[1])
    PARAM_ADD_CORE(PARAM_UINT16, m3, &motorPowerSet[2])
    PARAM_ADD_CORE(PARAM_UINT16, m4, &motorPowerSet[3])
PARAM_GROUP_STOP(motorPowerSet)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(memTst)
    PARAM_ADD(PARAM_UINT8, resetW, &memTesterWriteReset)
PARAM_GROUP_STOP(memTst)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(pm)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, lowVoltage, 
        &powerMonitorTask.batteryLowVoltage)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, criticalLowVoltage, 
        &powerMonitorTask.batteryCriticalLowVoltage)
PARAM_GROUP_STOP(pm)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(deck)
    PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow, &flowDeckTask.didInit)
    PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &flowDeckTask.didInit)
    PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &zrangerTask.didInit)  
    PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &zrangerTask.didInit)
PARAM_GROUP_STOP(deck)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(syslink)
    PARAM_ADD(PARAM_UINT8, probe, &syslink_triggerDebugProbe)
PARAM_GROUP_STOP(syslink)

    //////////////////////////////////////////////////////////////////////////////

    PARAM_GROUP_START(crtpsrv)
    PARAM_ADD(PARAM_UINT16, echoDelay, &system_echoDelay)
PARAM_GROUP_STOP(crtpsrv)
