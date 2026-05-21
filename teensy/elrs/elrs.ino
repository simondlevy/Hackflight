/*
   Hackflight main sketch for Teensy

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
#include <firmware/quadcore.hpp>
#include <firmware/receiver.hpp>
using namespace hf;

static CRSFforArduino _crsf = CRSFforArduino(&Serial2);

static Receiver::Data _rxdata;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        _rxdata = Receiver::Data::update(
                _rxdata,
                _crsf.readRcChannel(3),
                _crsf.readRcChannel(1),
                _crsf.readRcChannel(2),
                _crsf.readRcChannel(4),
                _crsf.readRcChannel(5),
                millis());
    }
}

void Receiver::begin()
{
    if (!_crsf.begin()) {
        _crsf.end();
        Debugger::reportForever("Unable to start ELRS receiver");
    }

    _crsf.setRcChannelsCallback(onReceiveRcChannels);
}

auto Receiver::read() -> Receiver::Data
{
    _crsf.update();

    return _rxdata;
}
static QuadCore _core;

void setup()
{
    _core.begin();
}

void loop()
{
    _core.step();
}
