/*
   Hackflight main sketch for Teensy using ELRS receiver

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

static QuadCore _core;

static CRSFforArduino _crsf = CRSFforArduino(&Serial2);

static ReceiverData _rxdata;

static StabilizerPid _stabilizerPid;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        _rxdata = ReceiverData::update(
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
        Debugger::reportForever("Unable to start ELRS receiver");
    }
    _crsf.setRcChannelsCallback(onReceiveRcChannels);

    // Start core sensors
    _core.begin();
}

void loop()
{
    // Read core sensors and do sensor fusion
    _core.getState();

    // This will trigger onReceiveRcChannels() above
    _crsf.update();

    // Disable arming while gyro is calibrating
    _rxdata = _core.isGyroCalibrated ? _rxdata : ReceiverData();

    // Check receiver timeout
    _rxdata = ReceiverData::checkTimeout(_rxdata, millis());

    // Run stabilizer PID control
    _stabilizerPid = StabilizerPid::run(
            _stabilizerPid,
            !_rxdata.is_throttle_down,
            Timer::getDt(),
            _core.state,
            mksetpoint(_rxdata.axes));

    // Run motor mixer and motors
    _core.runMotors(_rxdata.is_armed, _stabilizerPid.setpoint);
}
