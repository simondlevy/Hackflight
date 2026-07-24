/* Hackflight ESP32 transmitter sketch
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
#include <firmware/espnow.hpp>

static const uint8_t kTransmitterAddress[6] = {0x00,0x4B,0x12,0xCD,0x9E,0x08};
static const uint8_t kDongleAddress[6] = {0xD4,0xD4,0xDA,0x83,0x97,0x90};

static UMS3 ums3;
static int color;

static void OnDataRecv(
        const uint8_t * mac, const uint8_t * data, int len)
{
    (void)mac;

    Serial.println(len);
}

void setup()
{
    Serial.begin(115200);

    // Initialize all board peripherals, call this first
    ums3.begin();

    // Brightness is 0-255. We set it to 1/3 brightness here
    ums3.setPixelBrightness(255 / 3);

    // Enable the power to the RGB LED.
    // Off by default so it doesn't use current when the LED is not required.
    ums3.setPixelPower(true);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kTransmitterAddress);
    hf::EspNow::WifiAddPeer(kDongleAddress);

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop()
{
    const uint8_t data = 'A';

    if (esp_now_send(kDongleAddress, &data, 1) != ESP_OK) {
        Serial.println("Error sending the data");
    }

    // colorWheel cycles red, orange, ..., back to red at 256
    ums3.setPixelColor(UMS3::colorWheel(color));
    color++;

    delay(10);
}
