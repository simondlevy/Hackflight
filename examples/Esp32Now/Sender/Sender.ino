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

#include <hackflight.h>
#include <task/receiver/sbus.h>

#include <esp_now.h>
#include <WiFi.h>

#include <string.h>

// REPLACE WITH THE MAC Address of your receiver 
static uint8_t receiverAddress[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x69, 0xA0};

// Callback when data is sent
static const uint8_t RX_PIN = 4;
static const uint8_t TX_PIN = 14; // unused

static SbusReceiver _rx;

static uint16_t convert(uint16_t chanval)
{
    return (uint16_t)SbusReceiver::convert(chanval);
}

static void report (
        const uint16_t value, const char * label, const char * delim="   ")
{
    Serial.print(label);
    Serial.print(value);
    Serial.print(delim);
}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{

  Serial.begin(115200);

  // Start receiver
  Serial1.begin(100000, SERIAL_8E2, RX_PIN, TX_PIN, true);

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
    _rx.read(Serial1);

    if (_rx.ready()) {

        const uint16_t c1 = convert(_rx.readChannel1());
        const uint16_t c2 = convert(_rx.readChannel2());
        const uint16_t c3 = convert(_rx.readChannel3());
        const uint16_t c4 = convert(_rx.readChannel4());
        const uint16_t c5 = convert(_rx.readChannel5());
        const uint16_t c6 = convert(_rx.readChannel6());
        
        report(c1, "C1=");
        report(c2, "C2=");
        report(c3, "C3=");
        report(c4, "C4=");
        report(c5, "C5=");
        report(c6, "C6=", "\n");
    }

    /*
  static const char * message = "hello how are you";

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)message,
          strlen(message));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }*/
  
  delay(5);
}
