/*
   Copyright (c) 2023 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#include <espnow.h>

// Replace with the MAC Address of your sender 
static EspNow _esp = EspNow(0xD8, 0xA0, 0x1D, 0x62, 0xD4, 0xF0);

void setup(void)
{
    Serial.begin(115200);

    _esp.begin();

    esp_now_register_recv_cb(onDataRecv);
}

static void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
    (void)mac;

    for (auto k=0; k<len; ++k) {
        Serial.println(incomingData[k]);
    }
}

 
void loop(void)
{
    delay(1);
}
