/*
 * Hackflight ESPNOW radio dongle sketch
 *
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <espnow-transceiver.h>

static const uint8_t XIAO_ADDRESS[] = {0x8C,0xBF,0xEA,0xCB,0x8F,0x94};


void EspNowTransceiver::recv(const uint8_t * data, const uint8_t len)
{
    (void)data;
    (void)len;
}

void setup()
{
    Serial.begin(115200);

    EspNowTransceiver::begin(XIAO_ADDRESS);
}

void loop()
{
    const auto avail = Serial.available();
    uint8_t buf[256] = {};
    Serial.read(buf, avail);
    EspNowTransceiver::send(buf, avail);
}
