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

//   Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/

#include <esp_now.h>
#include <WiFi.h>

#include <string.h>

// REPLACE WITH THE MAC Address of your receiver 
static uint8_t receiverAddress[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x69, 0xA0};

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      while (true) {
          Serial.println("Failed to add peer");
      }
      return;
  }

  // Register for a callback function that will be called when data is received
  //esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  static const char * message = "hello how are you";

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)message,
          strlen(message));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  delay(1000);
}
