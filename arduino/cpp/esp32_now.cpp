/*
 * ESP32 NOW support
 *
 * Copyright (c) 2021 Simon D. Levy
 *
 * MIT license
 */

#include <esp_now.h>
#include <WiFi.h>
#include "arduino_debugger.hpp"

void esp32nowStart(void)
{
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Debugger::reportForever("Error initializing ESP-NOW");
    }
}

void esp32nowRegisterPeer(
        uint8_t mac1,
        uint8_t mac2,
        uint8_t mac3,
        uint8_t mac4,
        uint8_t mac5,
        uint8_t mac6)
{
    // Register peer
    esp_now_peer_info_t peerInfo;
    uint8_t peerAddress[6] = {mac1, mac2, mac3, mac4, mac5, mac6};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Debugger::reportForever("Failed to add peer");
    }
}
