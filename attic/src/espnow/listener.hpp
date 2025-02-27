/*
   ESP-NOW listener class declaration

   Copyright (C) 2024 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include <esp_now.h>
#include <WiFi.h>

namespace hf {

    class EspNowListener {

        public:

            virtual void espnow_listener_callback(
                    const uint8_t * data, const uint8_t len) = 0;
    };

}
