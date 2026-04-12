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

// callback function that will be executed when data is received
static void OnDataRecv(
        const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (int k=0; k<len; ++k) {
        Serial.println((char)incomingData[k], HEX);
    }
}

void setup()
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        while (true) {
            Serial.println("Error initializing ESP-NOW");
            delay(500);
        }
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{

}
