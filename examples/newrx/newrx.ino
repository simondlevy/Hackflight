/* Hackflight ESP32 receiver sketch
 * 
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, in version 3.  This program is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.  You should have received a copy of
 * the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <hackflight.h>
#include <firmware/blink_timer.hpp>

#include <firmware/espnow.hpp>

static const uint8_t kDongleAddress[6] = {
    
    // M5 Stack Atom
    0xD4,0xD4,0xDA,0x83,0x97,0x90

    // TinyS3
    //0xB4, 0x3A, 0x45, 0xB1, 0xF2, 0x40
};


static const uint32_t kSendDelayMsec = 5;
static const uint32_t kWifiTimeoutMsec = 50;

static const uint32_t kSerialBaudRate = 115'200;
static const uint8_t kSerialRxPin = 4;
static const uint8_t kSerialTxPin = 14;


void serialEvent1()
{
    while (Serial1.available()) {

        const uint8_t data = Serial1.read();

        const auto result = esp_now_send(kDongleAddress, &data, 1);

        if (result != ESP_OK) {
            Serial.printf("ERROR sending to dongle: %d\n", result);
        }

        delay(kSendDelayMsec);
    }
}


void setup()
{
    Serial.begin(115200);

    Serial1.begin(kSerialBaudRate, SERIAL_8N1, kSerialRxPin, kSerialTxPin);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kDongleAddress);
}

void loop()
{
    /*
       if (millis() - last_wifi_received_msec_ < kWifiTimeoutMsec) {
       ums3_.setPixelColor(0, 255, 0);
       }

    // not connected
    else {
    ums3_.setPixelColor(blink_timer_.On() ? 255 : 0, 0, 0);
    }*/

    /*
    static uint8_t k_;
    const uint8_t data = 'A' + k_;
    k_ = (k_ + 1) % 26;

    const auto result = esp_now_send(kDongleAddress, &data, 1);

    if (result != ESP_OK) {
        Serial.printf("ERROR sending to dongle: %d\n", result);
    }

    delay(kDelayMsec);*/
}

