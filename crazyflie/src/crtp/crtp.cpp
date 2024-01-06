/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * crtp.c - CrazyRealtimeTransferProtocol stack
 */


#include "crtp.h"

#include <radiolink.hpp>
#include <tasks/usblink.hpp>

extern RadioLink radioLink;
extern UsbLinkTask usbLinkTask;

// Shared with logger
crtpStats_t crtpStats;

static bool didInit;

static crtpLink_e linkType;

#define STATS_INTERVAL 500

static xQueueHandle  txQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 120
#define CRTP_RX_QUEUE_SIZE 16

static const size_t CRTP_TX_TASK_STACKSIZE  = configMINIMAL_STACK_SIZE;
static const size_t CRTP_RX_TASK_STACKSIZE  = 2 * configMINIMAL_STACK_SIZE;

static xQueueHandle queues[CRTP_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];
static void updateStats();

StackType_t  txTaskStackBuffer[CRTP_TX_TASK_STACKSIZE]; 
StaticTask_t txTaskTaskBuffer;

StackType_t rxTaskStackBuffer[CRTP_RX_TASK_STACKSIZE]; 
StaticTask_t rxTaskTaskBuffer;

static void clearStats()
{
    crtpStats.rxCount = 0;
    crtpStats.txCount = 0;
}

static void updateStats()
{
    uint32_t now = xTaskGetTickCount();
    if (now > crtpStats.nextStatisticsTime) {
        float interval = now - crtpStats.previousStatisticsTime;
        crtpStats.rxRate = (uint16_t)(1000.0f * crtpStats.rxCount / interval);
        crtpStats.txRate = (uint16_t)(1000.0f * crtpStats.txCount / interval);

        clearStats();
        crtpStats.previousStatisticsTime = now;
        crtpStats.nextStatisticsTime = now + STATS_INTERVAL;
    }
}

static void txTask(void *param)
{
    crtpPacket_t p;

    while (true) {

        if (linkType != CRTP_LINK_NONE) {

            if (xQueueReceive(txQueue, &p, portMAX_DELAY) == pdTRUE) {

                while (true) {

                    auto done = (linkType == CRTP_LINK_RADIO) ?
                        radioLink.sendPacket(&p) :
                        usbLinkTask.sendPacket(&p);

                    if (done) {
                        break;
                    }

                    // Relaxation time
                    vTaskDelay(M2T(10));
                }

                crtpStats.txCount++;
                updateStats();
            }
        }
        else {
            vTaskDelay(M2T(10));
        }
    }
}

static void rxTask(void *param)
{
    crtpPacket_t p;

    while (true) {

        if (linkType != CRTP_LINK_NONE) {

            auto done = (linkType == CRTP_LINK_RADIO) ?
                radioLink.receivePacket(&p) :
                usbLinkTask.receivePacket(&p);


            if (!done) {

                if (queues[p.port]) { // Block, since we should never drop a packet
                    xQueueSend(queues[p.port], &p, portMAX_DELAY);
                }

                if (callbacks[p.port]) { callbacks[p.port](&p);
                }

                crtpStats.rxCount++;
                updateStats();
            }
        }
        else {
            vTaskDelay(M2T(10));
        }
    }
}


//////////////////////////////////////////////////////////////////////////////

void crtpInit(void)
{
    if(didInit)
        return;

    txQueue = xQueueCreate(
            CRTP_TX_QUEUE_SIZE, 
            sizeof(crtpPacket_t));

    xTaskCreateStatic(
            txTask, 
            "CRTP-TX", 
            CRTP_TX_TASK_STACKSIZE,
            NULL, 
            2,
            txTaskStackBuffer,
            &txTaskTaskBuffer);

    xTaskCreateStatic(
            rxTask, 
            "CRTP-RX", 
            CRTP_RX_TASK_STACKSIZE,
            NULL, 
            2,
            rxTaskStackBuffer,
            &rxTaskTaskBuffer);

    didInit = true;
}

bool crtpTest(void)
{
    return didInit;
}

void crtpInitTaskQueue(CRTPPort portId)
{
    queues[portId] = xQueueCreate(
            CRTP_RX_QUEUE_SIZE, 
            sizeof(crtpPacket_t));
}

int crtpReceivePacket(CRTPPort portId, crtpPacket_t *p)
{
    return xQueueReceive(queues[portId], p, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, crtpPacket_t *p)
{
    return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int crtpReceivePacketWait(CRTPPort portId, crtpPacket_t *p, int wait)
{
    return xQueueReceive(queues[portId], p, M2T(wait));
}

int crtpGetFreeTxQueuePackets(void)
{
    return (CRTP_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}

void crtpRegisterPortCB(int port, CrtpCallback cb)
{
    if (port>CRTP_NBR_OF_PORTS)
        return;

    callbacks[port] = cb;
}

int crtpSendPacket(crtpPacket_t *p)
{
    return xQueueSend(txQueue, p, 0);
}

int crtpSendPacketBlock(crtpPacket_t *p)
{
    return xQueueSend(txQueue, p, portMAX_DELAY);
}

int crtpReset(void)
{
  xQueueReset(txQueue);
  return 0;
}

bool crtpIsConnected(void)
{                    
    return linkType == CRTP_LINK_RADIO ? radioLink.isConnected() : true;
}

void crtpSetLink(crtpLink_e linktype)
{
    linkType = linktype;
}
