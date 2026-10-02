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

#include <UMS3.h>

#include <hackflight.h>
#include <firmware/blink_timer.hpp>

#include <firmware/espnow.hpp>

static const uint8_t kDongleAddress[6] = {
    0x00, 0x4B, 0x12, 0xCD, 0x9B, 0xD0
};

// Maximum wifi receive failure time before giving up
static const uint32_t kWifiReceiveTimeoutMsec = 50;

// Maximum wifi send failure time before giving up
static const uint32_t kWifiSendTimeoutMsec = 2500;

// Serial comms with Teensy
static const uint32_t kSerialBaudRate = 115'200;
static const uint8_t kSerialRxPin = 14;
static const uint8_t kSerialTxPin = 13;

static UMS3 ums3_;

static uint32_t last_wifi_received_msec_;
static uint32_t last_wifi_sent_msec_;

// Relay transmitter Wifi input to Teensy over UART
static void OnWifiDataReceived(
        const uint8_t * mac, const uint8_t * data, int len)
{
    (void)mac;

    Serial1.write(data, len);

    last_wifi_received_msec_ = millis();
}

void OnWifiDataSent(const uint8_t * mac, esp_now_send_status_t status)
{
    (void)mac;

    if (status == ESP_OK) {
        last_wifi_sent_msec_ = millis();
    }
}

// Relay UART input from Teensy to dongle
void serialEvent1()
{
    const auto avail = Serial1.available();

    uint8_t buf[256] = {};

    Serial1.read(buf, avail);

    static bool should_give_up_;

    if (!should_give_up_) {
       esp_now_send(kDongleAddress, buf, avail);
    }

    // Don't keep trying to send to dongle if we haven't succeeded recently
    if ((millis()-last_wifi_sent_msec_) > kWifiSendTimeoutMsec) {
        should_give_up_ = true;
    }
}

void setup()
{
    // For debugging
    Serial.begin(115200);

    // Start serial comms with Teensy
    Serial1.begin(kSerialBaudRate, SERIAL_8N1, kSerialRxPin, kSerialTxPin);

    // Start RGB LED, dimming to 1/3 power
    ums3_.begin();
    ums3_.setPixelBrightness(255 / 3);
    ums3_.setPixelPower(true);

    // Start ESP comms
    hf::EspNow::WifiSetup();

    // Add dongle as Wifi peer to which we will send 
    hf::EspNow::WifiAddPeer(kDongleAddress);

    // Register Wifi data received from transmitter
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnWifiDataReceived));

    esp_now_register_send_cb(esp_now_send_cb_t(OnWifiDataSent));
}

void loop()
{
    static hf::BlinkTimer blink_timer_;

    // If we've received transmitter Wifi data recently, make LED solid green
    if (millis() - last_wifi_received_msec_ < kWifiReceiveTimeoutMsec) {
        ums3_.setPixelColor(0, 255, 0);
    }

    // Otherise, blink LED red
    else {
        ums3_.setPixelColor(blink_timer_.On() ? 255 : 0, 0, 0);
    }
}
