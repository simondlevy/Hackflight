/*
   Hackflight ESPNOW dongle sketch

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

// REPLACE WITH YOUR RECEIVER MAC Address
static const uint8_t RECEIVER_ADDRESS[] = {0x54, 0x32, 0x04, 0x33, 0x0D, 0xF0};

void serialEvent()
{
    while (Serial.available()) {
        const uint8_t c = Serial.read();
        esp_now_send(RECEIVER_ADDRESS, &c, 1);
    }
}

void setup()
{
    // Init Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_ADDRESS, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
}

void loop()
{
}
