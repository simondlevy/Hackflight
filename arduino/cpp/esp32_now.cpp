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

void esp32nowAddPeer(
        uint8_t mac1,
        uint8_t mac2,
        uint8_t mac3,
        uint8_t mac4,
        uint8_t mac5,
        uint8_t mac6)
{
    // Register peer
    esp_now_peer_info_t peerInfo = {};
    uint8_t peerAddress[6] = {mac1, mac2, mac3, mac4, mac5, mac6};
    memcpy(peerInfo.peer_addr, peerAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Debugger::reportForever("Failed to add peer");
    }
}

static uint8_t rxaddr[6];

void esp32nowPrepareToSend(
        uint8_t mac1,
        uint8_t mac2,
        uint8_t mac3,
        uint8_t mac4,
        uint8_t mac5,
        uint8_t mac6)
{
    rxaddr[0] = mac1;
    rxaddr[1] = mac2;
    rxaddr[2] = mac3;
    rxaddr[3] = mac4;
    rxaddr[4] = mac5;
    rxaddr[5] = mac6;
}

void commsWrite(uint8_t * buff, uint8_t size)
{
    esp_err_t result = esp_now_send(rxaddr, buff, size);
}
