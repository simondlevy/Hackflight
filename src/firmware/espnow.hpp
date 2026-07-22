/* Hackflight ESP NOW utilities
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <esp_now.h>
#include <WiFi.h>

#include <hackflight.h>
#include <firmware/debugger.hpp>

namespace hf {

    class EspNow {

        public:

            static void WifiSetup()
            {
                WiFi.mode(WIFI_STA);

                if (esp_now_init() != ESP_OK) {
                    hf::Debugger::ReportForever("Error initializing ESP-NOW");
                }
            }

            static void WifiAddPeer(const uint8_t addr[6])
            {
                esp_now_peer_info_t peerInfo = {};
                memcpy(peerInfo.peer_addr, addr, 6);
                peerInfo.channel = 0;
                peerInfo.encrypt = false;
            
                if (esp_now_add_peer(&peerInfo) != ESP_OK){
                    hf::Debugger::ReportForever("Failed to add peer");
                }
            }

    }; // class EspNow

} // namespace hf
