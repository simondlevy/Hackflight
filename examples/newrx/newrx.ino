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

#include <TinyPICO.h>

#include <hackflight.h>
#include <firmware/blink_timer.hpp>

#include <firmware/espnow.hpp>

static const uint8_t kDongleAddress[6] = {
    
    // M5 Stack Atom
    0xD4,0xD4,0xDA,0x83,0x97,0x90
};

static const uint32_t kSendDelayMsec = 5;
static const uint32_t kWifiTimeoutMsec = 50;

static const uint32_t kSerialBaudRate = 115'200;

static const uint8_t kSerial1RxPin = 4;
static const uint8_t kSerial1TxPin = 14;

static const uint8_t kSerial2RxPin = 5;
static const uint8_t kSerial2TxPin = 27;

static uint32_t last_wifi_received_msec_;

static TinyPICO tp_ = TinyPICO();

// Receive commands from transmitter; forward to Teensy over Serial1
static void OnWifiDataReceive(
        const uint8_t * mac, const uint8_t * data, int len)
{
    (void)mac;

    Serial1.write(data, len);

    last_wifi_received_msec_ = millis();

    delay(1);
}


// Receive telemetry from Teensy over Serial2; send over Wifi to dongle
static TaskHandle_t TelemetryTaskHandle = NULL;
static void TelemetryTask(void *parameter)
{
    while (true) {

        while (Serial2.available()) {
            const uint8_t data = Serial2.read();
            const auto result = esp_now_send(kDongleAddress, &data, 1);
            vTaskDelay(1);
        }

        const auto lag = millis() - last_wifi_received_msec_;

        // connected
        if (lag < kWifiTimeoutMsec) {
            tp_.DotStar_SetPixelColor( 0, 255/3, 0 );

        }

        // not connected
        else {
            tp_.DotStar_SetPixelColor( 255/3, 0, 0 );
        }

    }
}


void setup()
{
    Serial.begin(115200);

    Serial1.begin(kSerialBaudRate, SERIAL_8N1, kSerial1RxPin, kSerial1TxPin);
    Serial2.begin(kSerialBaudRate, SERIAL_8N1, kSerial2RxPin, kSerial2TxPin);


    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kDongleAddress);

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnWifiDataReceive));

    xTaskCreatePinnedToCore(
            TelemetryTask,         // Task function
            "TelemetryTask",       // Task name
            10000,             // Stack size (bytes)
            NULL,              // Parameters
            1,                 // Priority
            &TelemetryTaskHandle,  // Task handle
            1                  // Core 1
            );
}

void loop()
{
    /*
       delay(1);*/
}

