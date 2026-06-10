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

// Hackflight library
#include <hackflight.h>
#include <firmware/fc.hpp>
#include <firmware/effectors/quad_dshot.hpp>

static CRSFforArduino _crsf = CRSFforArduino(&Serial2);

static hf::FlightController _fc;

static hf::TraditionalReceiver _rxdata;

static hf::QuadDshot _effector;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        _rxdata = hf::TraditionalReceiver::Update(
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
    _fc.Begin();

    // Start motors
    _effector.Begin();
}

void loop()
{
    // This will trigger onReceiveRcChannels() above
    _crsf.update();

    // Run core algorithm to get setpoint from PID controllers
    const auto setpoint = _fc.Update(_rxdata, _effector.GetMotorValues(), 4);

    // Run sensor fusion on hover-deck
    _fc.AcquireHoverData();

    // Run the mixer and motors
    _effector.Run(_fc, setpoint);
}
