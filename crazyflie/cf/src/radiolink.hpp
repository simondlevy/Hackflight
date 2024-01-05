/*
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
 * radiolink.c - Radio link layer
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <free_rtos.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <crtp/crtp.h>

#include <tasks/syslink.hpp>

#include <config.h>
#include <configblock.hpp>
#include <radiolink.hpp>
#include <cfassert.h>
#include <ledseq.h>

class RadioLink {

    public:

        // Shared with logger
        uint8_t rssi;
        bool isConnectedFlag;

        void init(ConfigBlock & configBlock)
        {
            if (didInit)
                return;

            txQueue = xQueueCreateStatic(
                    TX_QUEUE_LENGTH,
                    TX_ITEM_SIZE,
                    txQueueStorage,
                    &txQueueBuffer);

            rxQueue = xQueueCreateStatic(
                    RX_QUEUE_LENGTH,
                    RX_ITEM_SIZE,
                    rxQueueStorage,
                    &rxQueueBuffer);

            syslinkInit();

            setChannel(configBlock.getRadioChannel());
            setDatarate(configBlock.getRadioSpeed());
            setAddress(configBlock.getRadioAddress());

            didInit = true;
        }

        void setPowerDbm(int8_t powerDbm)
        {
            syslinkPacket_t slp;

            slp.type = SYSLINK_RADIO_POWER;
            slp.length = 1;
            slp.data[0] = powerDbm;
            syslinkSendPacket(&slp);
        }

        bool test(void)
        {
            return syslinkTest();
        }

        void syslinkDispatch(syslinkPacket_t *slp)
        {
            static syslinkPacket_t txPacket;

            if (slp->type == SYSLINK_RADIO_RAW || slp->type == SYSLINK_RADIO_RAW_BROADCAST) {
                lastPacketTick = xTaskGetTickCount();
            }

            if (slp->type == SYSLINK_RADIO_RAW)
            {
                slp->length--; // Decrease to get CRTP size.
                // Assert that we are not dropping any packets
                ASSERT(xQueueSend(rxQueue, &slp->length, 0) == pdPASS);
                ledseqShowLinkUp();
                // If a radio packet is received, one can be sent
                if (xQueueReceive(txQueue, &txPacket, 0) == pdTRUE) {
                    ledseqShowLinkDown();
                    syslinkSendPacket(&txPacket);
                }
            } 

            else if (slp->type == SYSLINK_RADIO_RAW_BROADCAST) {
                slp->length--; // Decrease to get CRTP size.
                // broadcasts are best effort, so no need to handle the case
                // where the queue is full
                xQueueSend(rxQueue, &slp->length, 0);
                ledseqShowLinkUp();
                // no ack for broadcasts
            } 

            else if (slp->type == SYSLINK_RADIO_RSSI) {
                //Extract RSSI sample sent from radio
                memcpy(&rssi, slp->data, //rssi will not change on disconnect
                        sizeof(uint8_t)); 
            } 

            else if (slp->type == SYSLINK_RADIO_P2P_BROADCAST) {
                ledseqShowLinkUp();
                P2PPacket p2pp;
                p2pp.port=slp->data[0];
                p2pp.rssi = slp->data[1];

                const uint8_t p2pDataLength = slp->length - 2;
                memcpy(&p2pp.data[0], &slp->data[2], p2pDataLength);
                p2pp.size = p2pDataLength;
            }

            isConnectedFlag = isConnected();
        }

        bool isConnected(void)
        {
            return (xTaskGetTickCount() - lastPacketTick) < M2T(ACTIVITY_TIMEOUT_MS);
        }

        int sendPacket(crtpPacket_t *p)
        {
            static syslinkPacket_t slp;

            ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

            slp.type = SYSLINK_RADIO_RAW;
            slp.length = p->size + 1;
            memcpy(slp.data, &p->header, p->size + 1);

            return xQueueSend(txQueue, &slp, M2T(100)) == pdTRUE;
        }

        int receivePacket(crtpPacket_t *p)
        {
            return xQueueReceive(rxQueue, p, M2T(100)) == pdTRUE ? 0 : -1;
        }

    private:

        static const uint8_t P2P_CRTP_MAX_DATA_SIZE = 60;
        static const uint8_t P2P_QUEUE_SIZE = 5;

        typedef struct _P2PPacket
        {
            uint8_t size;                         //< Size of data
            uint8_t rssi;                         //< Received Signal Strength Intensity
            union {
                struct {
                    uint8_t port;                 //< Header selecting channel and port
                    uint8_t data[P2P_CRTP_MAX_DATA_SIZE]; //< Data
                };
                uint8_t raw[P2P_CRTP_MAX_DATA_SIZE+1];  //< The full packet "raw"
            };
        } __attribute__((packed)) P2PPacket;

        typedef void (*P2PCallback)(P2PPacket *);


        static const uint32_t ACTIVITY_TIMEOUT_MS = 1000;

        static const uint8_t TX_QUEUE_LENGTH = 1;
        static const auto TX_ITEM_SIZE = sizeof(syslinkPacket_t);
        uint8_t txQueueStorage[TX_QUEUE_LENGTH * TX_ITEM_SIZE];
        StaticQueue_t txQueueBuffer;
        xQueueHandle  txQueue;

        static const uint8_t RX_QUEUE_LENGTH = 5;
        static const auto RX_ITEM_SIZE = sizeof(crtpPacket_t);
        uint8_t rxQueueStorage[RX_QUEUE_LENGTH * RX_ITEM_SIZE];
        StaticQueue_t rxQueueBuffer;
        xQueueHandle rxQueue;

        bool didInit;

        uint32_t lastPacketTick;

        static void setChannel(uint8_t channel)
        {
            syslinkPacket_t slp;

            slp.type = SYSLINK_RADIO_CHANNEL;
            slp.length = 1;
            slp.data[0] = channel;
            syslinkSendPacket(&slp);
        }

        static void setDatarate(uint8_t datarate)
        {
            syslinkPacket_t slp;

            slp.type = SYSLINK_RADIO_DATARATE;
            slp.length = 1;
            slp.data[0] = datarate;
            syslinkSendPacket(&slp);
        }

        static void setAddress(uint64_t address)
        {
            syslinkPacket_t slp;

            slp.type = SYSLINK_RADIO_ADDRESS;
            slp.length = 5;
            memcpy(&slp.data[0], &address, 5);
            syslinkSendPacket(&slp);
        }


};
