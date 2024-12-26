/*
   ESP-NOW listener implementation

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

#include <hackflight.hpp>
#include <espnow/listener.hpp>
#include <espnow/utils.hpp>

static hf::EspNowListener * _listener;

static void _callback(const uint8_t * mac, const uint8_t * data, int len) 
{
    _listener->espnow_listener_callback(data, len);
}

void hf::EspNowUtils::set_listener_callback(hf::EspNowListener * listener)
{
    _listener = listener;

    esp_now_register_recv_cb(esp_now_recv_cb_t(_callback));
}
