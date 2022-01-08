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

void esp32nowSend(
          uint8_t rxmac1,
        , uint8_t rxmac2,
        , uint8_t rxmac3,
        , uint8_t rxmac4,
        , uint8_t rxmac5,
        , uint8_t rxmac6,
        , uint8_t hdr0
        , uint8_t hdr1
        , uint8_t hdr2
        , uint8_t hdr3
        , uint8_t hdr4
        , uint8_t crc
        , uint8_t size
        , float val00
        , float val01
        , float val02
        , float val03
        , float val04
        , float val05
        )
{
    uint8_t rxAddress[6] = {rxmac1, rxmac2, rxmac3, rxmac4, rxmac5, rxmac6};
    //esp_err_t result = esp_now_send(rxAddress, &foo, 1);
}
