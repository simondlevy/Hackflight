/*
   Hackflight ESPNOW onboard radio sketch

   Copyright (C) 2026 Simon D. Levy

   Based on: https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files.  The above copyright
   notice and this permission notice shall be included in all copies or
   substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

#include <hackflight.h>
#include <firmware/espnow.h>

// Create a espnow_message_t called myData
static espnow_message_t myData;

// callback function that will be executed when data is received
static void OnDataRecv(
        const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
  Serial.print("Bool: ");
  Serial.println(myData.d);
  Serial.println();
}
 
void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop()
{

}
