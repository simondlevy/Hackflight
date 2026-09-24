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
#include <firmware/msp/__messages__.h>
#include <firmware/msp/parser.hpp>

static const uint8_t kTransmitterAddress[6] = {0xB4, 0x3A, 0x45, 0xB2, 0x08, 0x40};

static const uint8_t kDongleAddress[6] = {0xD4,0xD4,0xDA,0x83,0x97,0x90};

static const uint32_t kTimeoutMsec = 50;

static const uint32_t kSerialBaudRate = 115'200;
static const uint8_t kSerialRxPin = 44;
static const uint8_t kSerialTxPin = 43;

static const uint32_t kDelayMsec = 10;


static auto blink_timer_ = hf::BlinkTimer();

static UMS3 ums3_;

static uint32_t last_received_msec_;

static void OnDataRecv(
        const uint8_t * mac, const uint8_t * data, int len)
{
    (void)mac;

    Serial1.write(data, len);

    last_received_msec_ = millis();
}

void setup()
{
    Serial.begin(115200);

    Serial1.begin(kSerialBaudRate, SERIAL_8N1, kSerialRxPin, kSerialTxPin);

    ums3_.begin();
    ums3_.setPixelBrightness(255 / 3);
    ums3_.setPixelPower(true);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kTransmitterAddress);
    hf::EspNow::WifiAddPeer(kDongleAddress);

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{

    if (millis() - last_received_msec_ < kTimeoutMsec) {
            ums3_.setPixelColor(0, 255, 0);
    }

    // not connected
    else {
        ums3_.setPixelColor(blink_timer_.On() ? 255 : 0, 0, 0);
    }

    delay(kDelayMsec);
}
