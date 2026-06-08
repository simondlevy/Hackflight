/*
   Hackflight main sketch for Teensy quadcopter using ELRS receiver without
   hover

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
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/fc.hpp>
#include <mixers/bfquadx.hpp>

static CRSFforArduino _crsf = CRSFforArduino(&Serial2);

static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

static hf::FC _fc;

static hf::TraditionalReceiver _rxdata;

static hf::Mixer _mixer;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        _rxdata = hf::TraditionalReceiver::update(
                _rxdata,
                _crsf.readRcChannel(3),
                _crsf.readRcChannel(1),
                _crsf.readRcChannel(2),
                _crsf.readRcChannel(4),
                _crsf.readRcChannel(5),
                millis());
    }
}

void setup()
{
    // Start receiver
    if (!_crsf.begin()) {
        _crsf.end();
        hf::Debugger::ReportForever("Unable to start ELRS receiver");
    }
    _crsf.setRcChannelsCallback(onReceiveRcChannels);

    // Start flight control
    _fc.begin();

    // Start motors
    _motors.begin();
}

void loop()
{
    // This will trigger onReceiveRcChannels() above
    _crsf.update();

    // Run core algorithm to get setpoint from PID controllers
    const auto setpoint = _fc.update(_rxdata, _mixer.motorvals, 4);

    // Run sensor fusion on hover-deck
    _fc.acquireHoverData();

    // Run motor mixer on setpoint
    _mixer = hf::Mixer::run(_mixer, setpoint);

    // Run motors if safe
    if (_fc.isSafeToFly()) {
        _motors.run(_fc.isArmed(), _mixer.motorvals);
    }
}
