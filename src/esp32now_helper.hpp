/*
   ESP32-NOW helper class

   Adapted from 

     https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <esp_now.h>
#include <WiFi.h>

class Esp32Now {

    public:

        typedef enum {

            ERROR_NONE,
            ERROR_INIT,
            ERROR_PEER

        } error_t;

        static error_t init(
                const uint8_t peer_address[6],
                void (*OnDataRecv)(
                    const uint8_t * mac, const uint8_t *incomingData, int len))
        {
            WiFi.mode(WIFI_STA);

            if (esp_now_init() != ESP_OK) {
                return ERROR_INIT;
            }

            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peer_address, 6);
            peerInfo.channel = 0;  
            peerInfo.encrypt = false;

            if (esp_now_add_peer(&peerInfo) != ESP_OK){
                return ERROR_PEER;
            }

            esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

            return ERROR_NONE;
        }

};
