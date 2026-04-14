/*
   Hackflight with ELRS receiver

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

// Third-party libraries
#include <CRSFforArduino.hpp>

// Hackflight library

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/rx.hpp>
using namespace hf;

static CRSFforArduino _crsf;

static RX::Data _rxdata;

static void onReceiveRcChannels(
        serialReceiverLayer::rcChannels_t *rcChannels, void * obj)
{
    if (!rcChannels->failsafe) {

        _rxdata = RX::Data::update(
                _rxdata, 
                _crsf.readRcChannel(3),
                _crsf.readRcChannel(1),
                _crsf.readRcChannel(2),
                _crsf.readRcChannel(4),
                _crsf.readRcChannel(5),
                millis());
    }
}

void RX::begin()
{
    _crsf = CRSFforArduino(&Serial5);

    if (!_crsf.begin()) {
        _crsf.end();
        Debugger::reportForever("CRSF for Arduino initialisation failed!");
    }

    _crsf.setRcChannelsCallback(onReceiveRcChannels, nullptr);

}

auto RX::read() -> RX::Data
{

    _crsf.update();

    return _rxdata;
}
