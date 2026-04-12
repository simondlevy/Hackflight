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

// REPLACE WITH YOUR RECEIVER MAC Address
static const uint8_t RECEIVER_ADDRESS[] = {0x54, 0x32, 0x04, 0x33, 0x0D, 0xF0};

void serialEvent()
{
    while (Serial.available()) {
        const uint8_t c = Serial.read();
        esp_now_send(RECEIVER_ADDRESS, &c, 1);
    }
}

static void reportForever(const char * msg)
{
    while (true) {
        Serial.println(msg);
        delay(500);
    }
}

void setup()
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        reportForever("Error initializing ESP-NOW");
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_ADDRESS, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        reportForever("Failed to add peer");
    }
}

void loop()
{
}
