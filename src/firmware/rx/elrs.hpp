/*
   Copyright (C) 2026 Simon D. Levy

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

#include <CRSFforArduino.hpp>

static constexpr uint32_t RX_TIMEOUT_USEC = 500'000;

static CRSFforArduino _crsf;

static float _rx_chanvals[5];

static constexpr int CHANNEL_COUNT = 5;

static auto scalechan(serialReceiverLayer::rcChannels_t *rcChannels,
        const int k) -> float
{
    return map((float)_crsf.readRcChannel(k), 989, 2012, -1, +1);
}

static uint32_t _last_rx_usec;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        _rx_chanvals[0] = scalechan(rcChannels, 3);
        _rx_chanvals[1] = scalechan(rcChannels, 1);
        _rx_chanvals[2] = scalechan(rcChannels, 2);
        _rx_chanvals[3] = scalechan(rcChannels, 4);
        _rx_chanvals[4] = scalechan(rcChannels, 5);

        _last_rx_usec = micros();
    }
}

static void rx_init()
{
    if (!_crsf.begin()) {

        _crsf.end();

        while (true) {
            printf("CRSF for Arduino initialisation failed!\n");
            delay(500);
        }
    }

    _crsf.setRcChannelsCallback(onReceiveRcChannels);
}
