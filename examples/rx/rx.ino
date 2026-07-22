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

#include <hackflight.h>
#include <firmware/espnow.hpp>

static const uint8_t kTransmitterAddress[6] = {0xD4,0xD4,0xDA,0xAA,0x2E,0xF0};
static const uint8_t kDongleAddress[6] = {0xD4,0xD4,0xDA,0x83,0x97,0x90};

void setup()
{
    Serial.begin(115200);

    hf::EspNow::WifiSetup();
    hf::EspNow::WifiAddPeer(kTransmitterAddress);
    hf::EspNow::WifiAddPeer(kDongleAddress);
}

void loop()
{
}
