/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 */

#pragma once

#include <string.h>

#include <free_rtos.h>
#include <semphr.h>
#include <task.h>
#include <queue.h>

#include <crtp/crtp.h>

#include <hal/usb.h>

#include <tasks/usblink.hpp>
#include <tasks/syslink.hpp>

#include <cfassert.h>
#include <config.h>
#include <configblock.hpp>
#include <ledseq.h>

void usblinkInit();

struct crtpLinkOperations * usblinkGetLink();

class UsbLinkTask {

    public:

        void begin(void)
        {
            if(didInit)
                return;

            // Initialize the USB peripheral
            usbInit();

            packetsQueue = xQueueCreateStatic(
                    QUEUE_LENGTH, 
                    QUEUE_ITEM_SIZE,
                    queueStorage,
                    &queueBuffer);

            xTaskCreateStatic(
                    runTask, 
                    "USBLINK", 
                    configMINIMAL_STACK_SIZE, 
                    this, 
                    3, 
                    taskStackBuffer,
                    &taskTaskBuffer);

            didInit = true;
        }

        bool isConnected(void)
        {
            return true;
        }

        int receivePacket(crtpPacket_t *p)
        {
            if (xQueueReceive(packetsQueue, p, M2T(100)) == pdTRUE) {
                ledseqShowLinkUp();
                return 0;
            }

            return -1;
        }

        int sendPacket(crtpPacket_t *p)
        {
            ASSERT(p->size < SYSLINK_MTU);

            sendBuffer[0] = p->header;

            if (p->size <= CRTP_MAX_DATA_SIZE) {
                memcpy(&sendBuffer[1], p->data, p->size);
            }

            ledseqShowLinkDown();

            return usbSendData(p->size + 1, sendBuffer);
        }

    private:

        static const auto TASK_STACK_DEPTH = configMINIMAL_STACK_SIZE;
        StackType_t  taskStackBuffer[TASK_STACK_DEPTH]; 
        StaticTask_t taskTaskBuffer;

        static const auto QUEUE_ITEM_SIZE = sizeof(crtpPacket_t);
        static const size_t QUEUE_LENGTH = 16;
        uint8_t queueStorage[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
        StaticQueue_t queueBuffer;
        xQueueHandle packetsQueue;

        USBPacket usbIn;

        crtpPacket_t packet;

        bool didInit;

        uint8_t sendBuffer[64];

        static void runTask(void * param)
        {
            auto task = (UsbLinkTask *)param;

            while (true) {

                usbGetDataBlocking(&task->usbIn);

                task->packet.size = task->usbIn.size - 1;

                memcpy(&task->packet.raw, task->usbIn.data, task->usbIn.size);

                xQueueSend(task->packetsQueue, &task->packet, portMAX_DELAY);
            }
        }
};
