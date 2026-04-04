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

// Hackflight
#include <hackflight.h>
#include <firmware/rx.hpp>
using namespace hf;

static constexpr uint32_t ELRS_TIMEOUT_MSEC = 500;

static constexpr float THROTTLE_DOWN_MAX = -0.95;

static uint32_t _last_rx_msec;

static CRSFforArduino _crsf;

static RX::Data _data;

float scalechan(const uint8_t k)
{
    return map((float)_crsf.readRcChannel(k), 989, 2012, -1, +1);
}

static void onReceiveRcChannels(
        serialReceiverLayer::rcChannels_t *rcChannels, void * obj)
{
    if (!rcChannels->failsafe) {

        _data.axes.thrust = scalechan(3);
        _data.axes.roll = scalechan(1);
        _data.axes.pitch = scalechan(2);
        _data.axes.yaw = scalechan(4);

        _data.aux = scalechan(5);

        _last_rx_msec = millis();
    }
}

void RX::begin()
{
    _crsf = CRSFforArduino(_serial);

    if (!_crsf.begin()) {

        _crsf.end();

        while (true) {
            printf("CRSF for Arduino initialisation failed!\n");
            delay(500);
        }
    }

    _crsf.setRcChannelsCallback(onReceiveRcChannels, this);
}

auto RX::read() -> RX::Data
{
    _crsf.update();

    _data.is_throttle_down = _data.axes.thrust < THROTTLE_DOWN_MAX;

    const auto msec_curr = millis();

    // Check failsafe via RX timeout
    if (_last_rx_msec > 0 &&
            msec_curr > _last_rx_msec &&
            msec_curr - _last_rx_msec > ELRS_TIMEOUT_MSEC) {
        _data.is_armed = false;
    }

    // Push-button arming
    static float _chan5_prev;
    const auto chan5_curr = _data.aux;
    if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
        _data.is_armed =
            _data.is_armed ? false :
            _data.is_throttle_down ? true :
            _data.is_armed;
    }
    _chan5_prev = chan5_curr;

    return RX::Data(_data);
}
