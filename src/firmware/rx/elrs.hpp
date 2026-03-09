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

// "public"
static float rx_chanvals[5];
static bool rx_is_armed;
static bool rx_is_throttle_down;

static constexpr uint32_t ELRS_TIMEOUT_MSEC = 500;

static const float THROTTLE_DOWN_MAX = -0.95;

static CRSFforArduino _crsf;

static auto scalechan(serialReceiverLayer::rcChannels_t *rcChannels,
        const int k) -> float
{
    return map((float)_crsf.readRcChannel(k), 989, 2012, -1, +1);
}

static uint32_t _last_rx_msec;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        rx_chanvals[0] = scalechan(rcChannels, 3);
        rx_chanvals[1] = scalechan(rcChannels, 1);
        rx_chanvals[2] = scalechan(rcChannels, 2);
        rx_chanvals[3] = scalechan(rcChannels, 4);
        rx_chanvals[4] = scalechan(rcChannels, 5);

        _last_rx_msec = millis();
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

static void rx_read()
{
    _crsf.update();

    const auto throttle_is_down = rx_chanvals[0] < THROTTLE_DOWN_MAX;

    const auto msec_curr = millis();

    // Check failsafe via RX timeout
    if (_last_rx_msec > 0 &&
            msec_curr > _last_rx_msec &&
            msec_curr - _last_rx_msec > ELRS_TIMEOUT_MSEC) {
        rx_is_armed = false;
    }

    // Push-button arming
    static float _chan5_prev;
    const auto chan5_curr = rx_chanvals[4];
    if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
        rx_is_armed = rx_is_armed ? false : throttle_is_down ? true : rx_is_armed;
    }
    _chan5_prev = chan5_curr;
}

