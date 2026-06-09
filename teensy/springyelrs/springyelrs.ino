/*
   Hackflight main sketch for Teensy quadcopter using ELRS receiver with
   springy throttle

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

static hf::FC _fc;

static hf::SpringyReceiver _rxdata;

static hf::QuadDshot _effector;

static void onReceiveRcChannels(
        serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        const auto button_val = _crsf.readRcChannel(6);

        static uint16_t _button_val;

        static bool _aux2_on;

        if (button_val == 1000 && _button_val == 2000) {
            _aux2_on = !_aux2_on;
        }

        _button_val = button_val;

        _rxdata = hf::SpringyReceiver::Update(
                _rxdata,
                _crsf.readRcChannel(3),
                _crsf.readRcChannel(1),
                _crsf.readRcChannel(2),
                _crsf.readRcChannel(4),
                _crsf.readRcChannel(5),
                _aux2_on ? 2000 : 1000, 
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
    const auto setpoint = _fc.Update(_rxdata, _mixer.motorvals, 4);

    // Run the mixer and motors
    _effector.Run(_fc, setpoint);
}
