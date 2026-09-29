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

static const uint32_t kWifiTimeoutMsec = 50;

static auto blink_timer_ = hf::BlinkTimer();

static uint32_t last_wifi_received_msec_;

static int count_;

// Send Wifi input to Teensy over UART
static void OnWifiDataReceive(
        const uint8_t * mac, const uint8_t * data, int len)
{
    (void)mac;

    // Serial.write(data, len);

    count_ += len;

    last_wifi_received_msec_ = millis();
}


void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnWifiDataReceive));
}

void loop()
{
    if (millis() - last_wifi_received_msec_ < kWifiTimeoutMsec) {
        //ums3_.setPixelColor(0, 255, 0);
    }

    // not connected
    else {
        //ums3_.setPixelColor(blink_timer_.On() ? 255 : 0, 0, 0);
    }

    Serial.printf("count=%d\n", count_);
}
