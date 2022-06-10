/*
Copyright (c) 2022 Simon D. Levy

This file is part of Hackflight.

Hackflight is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Hackflight. If not, see <https://www.gnu.org/licenses/>.
*/


//  Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of your sender 
uint8_t senderAddress[] = {0x98, 0xCD, 0xAC, 0xD3, 0x42, 0x3C};

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    for (uint8_t k=0; k<len; ++k) {
        uint8_t byte = incomingData[k];
        printf("%s%02X ", byte==0x24 ? "\n" : "", byte);
    }

    delay(1);
}
 
void setup()
{

  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, senderAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop()
{
    delay(1);
}
