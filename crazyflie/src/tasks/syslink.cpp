#define DEBUG_MODULE "SL"

#include <free_rtos.h>
#include <task.h>
#include <timers.h>

#include <hal/uart_syslink.h>

#include <onewire.h>
#include <radiolink.hpp>

#include "power.hpp"
#include "syslink.hpp"

static const auto TASK_STACK_DEPTH = 2 * configMINIMAL_STACK_SIZE;
StackType_t  syslinkStackBuffer[TASK_STACK_DEPTH]; 
StaticTask_t syslinkTaskBuffer;


// Shared with params
uint8_t syslink_triggerDebugProbe;

static bool didInit;

static uint8_t sendBuffer[SYSLINK_MTU + 6];

static xTimerHandle debugTimer;
static xSemaphoreHandle syslinkAccess;

static void debugSyslinkReceive(syslinkPacket_t *slp) 
{
    if (slp->type == SYSLINK_DEBUG_PROBE) {
        consolePrintf("SYSLINK: NRF Address received: %d\n", slp->data[0]);
        consolePrintf("SYSLINK: NRF Chan received: %d\n", slp->data[1]);
        consolePrintf("SYSLINK: NRF Rate received: %d\n", slp->data[2]);
        consolePrintf("SYSLINK: NRF Dropped: %d\n", slp->data[3]);
        consolePrintf("SYSLINK: NRF uart error code: %d\n", slp->data[4]);
        consolePrintf("SYSLINK: NRF uart error count: %d\n", slp->data[5]);
        consolePrintf("SYSLINK: NRF uart checksum 1 fail count: %d\n", slp->data[6]);
        consolePrintf("SYSLINK: NRF uart checksum 2 fail count: %d\n", slp->data[7]);
    }
}

static void syslinkRouteIncomingPacket(syslinkPacket_t *slp)
{
    uint8_t groupType;

    groupType = slp->type & SYSLINK_GROUP_MASK;

    extern RadioLink radioLink;
    extern PowerMonitorTask powerMonitorTask;

    switch (groupType) {

        case SYSLINK_RADIO_GROUP:
            radioLink.syslinkDispatch(slp);
            break;
        case SYSLINK_PM_GROUP:
            powerMonitorTask.syslinkUpdate(slp);
            break;
        case SYSLINK_OW_GROUP:
            owSyslinkReceive(slp);
            break;
        case SYSLINK_SYS_GROUP:
            systemSyslinkReceive(slp);
            break;
        case SYSLINK_DEBUG_GROUP:
            debugSyslinkReceive(slp);
            break;
        default:
            consolePrintf("SYSLINK: Unknown packet:%X.\n", slp->type);
            break;
    }
}

static void syslinkTask(void *param)
{
    syslinkPacket_t slp = {};

    while (true) {
        uartslkGetPacketBlocking(&slp);
        syslinkRouteIncomingPacket(&slp);
    }
}


static void debugHandler(xTimerHandle timer) {
    static syslinkPacket_t txPacket;

    if (syslink_triggerDebugProbe) {
        syslink_triggerDebugProbe = 0;

        uartSyslinkDumpDebugProbe();
        consolePrintf("SYSLINK: Syslink NRF debug probe initialized\n");

        txPacket.type = SYSLINK_DEBUG_PROBE;
        txPacket.length = 0;
        syslinkSendPacket(&txPacket);
    }
}

//////////////////////////////////////////////////////////////////////////////

void syslinkInit()
{
    if(didInit) {
        return;
    }

    vSemaphoreCreateBinary(syslinkAccess);

    xTaskCreateStatic(
            syslinkTask, 
            "SYSLINK", 
            TASK_STACK_DEPTH,
            NULL, 
            3, 
            syslinkStackBuffer,
            &syslinkTaskBuffer);

    debugTimer = xTimerCreate( "syslinkTimer", M2T(1000), pdTRUE, NULL, debugHandler );
    xTimerStart(debugTimer, M2T(1000));

    didInit = true;
}

bool syslinkTest()
{
    return didInit;
}

int syslinkSendPacket(syslinkPacket_t *slp)
{
    int i = 0;
    int dataSize;
    uint8_t cksum[2] = {0};

    xSemaphoreTake(syslinkAccess, portMAX_DELAY);

    ASSERT(slp->length <= SYSLINK_MTU);

    sendBuffer[0] = SYSLINK_START_BYTE1;
    sendBuffer[1] = SYSLINK_START_BYTE2;
    sendBuffer[2] = slp->type;
    sendBuffer[3] = slp->length;

    memcpy(&sendBuffer[4], slp->data, slp->length);
    dataSize = slp->length + 6;
    // Calculate checksum delux
    for (i = 2; i < dataSize - 2; i++)
    {
        cksum[0] += sendBuffer[i];
        cksum[1] += cksum[0];
    }
    sendBuffer[dataSize-2] = cksum[0];
    sendBuffer[dataSize-1] = cksum[1];

    uartslkSendDataDmaBlocking(dataSize, sendBuffer);

    xSemaphoreGive(syslinkAccess);

    return 0;
}

