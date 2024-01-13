/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * platformservice.c - Implements platform services for CRTP
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <free_rtos.h>
#include <queue.h>
#include <semphr.h>

#include <tasks/core.hpp>

#include <tasks/syslink.hpp>

#include <crtp/crtp.h>

#include <platform/platform.h>

#include <safety.hpp>
#include <version.h>

class PlatformService {

    public:

        void task(Safety * safety)
        {
            static crtpPacket_t p;

            crtpInitTaskQueue(CRTP_PORT_PLATFORM);

            while (true) {

                crtpReceivePacketBlock(CRTP_PORT_PLATFORM, &p);

                switch (p.channel) {

                    case platformCommand:
                        platformCommandProcess(&p, safety);
                        crtpSendPacketBlock(&p);
                        break;
                    case versionCommand:
                        versionCommandProcess(&p);
                        break;
                    case appChannel:
                        appchannelIncomingPacket(&p);
                        break;
                    default:
                        break;
                }
            }
        }

        void init(void)
        {
            if (didInit) {
                return;
            }

            sendMutex = xSemaphoreCreateMutex();

            rxQueue = xQueueCreate(10, sizeof(crtpPacket_t));

            overflow = false;

            didInit = true;
        }

        bool test(void)
        {
            return didInit;
        }

    private:

        bool didInit;

        typedef enum {
            platformCommand   = 0x00,
            versionCommand    = 0x01,
            appChannel        = 0x02,
        } Channel;

        typedef enum {
            setContinuousWave  = 0x00,
            armSystem          = 0x01,
        } PlatformCommand;

        typedef enum {
            getProtocolVersion = 0x00,
            getFirmwareVersion = 0x01,
            getDeviceTypeName  = 0x02,
        } VersionCommand;

        SemaphoreHandle_t sendMutex;

        xQueueHandle  rxQueue;

        bool overflow;

        void appchannelIncomingPacket(crtpPacket_t *p)
        {
            int res = xQueueSend(rxQueue, p, 0);

            if (res != pdTRUE) {
                overflow = true;
            }
        }

        void platformCommandProcess(crtpPacket_t *p, Safety * safety)
        {
            uint8_t command = p->data[0];
            uint8_t *data = &p->data[1];

            switch (command) {
                case setContinuousWave:
                    {
                        static syslinkPacket_t slp;
                        slp.type = SYSLINK_RADIO_CONTWAVE;
                        slp.length = 1;
                        slp.data[0] = data[0];
                        syslinkSendPacket(&slp);
                        break;
                    }
                case armSystem:
                    {
                        const bool doArm = data[0];
                        const bool success = safety->requestArming(doArm);
                        data[0] = success;
                        data[1] = safety->isArmed();
                        p->size = 2;
                        break;
                    }
                default:
                    break;
            }
        }

        void versionCommandProcess(crtpPacket_t *p)
        {
            switch (p->data[0]) {
                case getProtocolVersion:
                    *(int*)&p->data[1] = CRTP_PROTOCOL_VERSION;
                    p->size = 5;
                    crtpSendPacketBlock(p);
                    break;
                case getFirmwareVersion:
                    strncpy((char*)&p->data[1], V_STAG, CRTP_MAX_DATA_SIZE-1);
                    p->size = (strlen(V_STAG)>CRTP_MAX_DATA_SIZE-1) ?
                        CRTP_MAX_DATA_SIZE:strlen(V_STAG)+1;
                    crtpSendPacketBlock(p);
                    break;
                case getDeviceTypeName:
                    {
                        const char* name = platformConfigGetDeviceTypeName();
                        strncpy((char*)&p->data[1], name, CRTP_MAX_DATA_SIZE-1);
                        p->size = (strlen(name)>CRTP_MAX_DATA_SIZE-1) ?
                            CRTP_MAX_DATA_SIZE:strlen(name)+1;
                        crtpSendPacketBlock(p);
                    }
                    break;
                default:
                    break;
            }
        }

}; // class PlatformService
