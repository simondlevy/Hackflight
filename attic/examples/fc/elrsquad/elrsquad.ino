/*
   Hackflight flight-controller sketch for Teensy quadcopter using ELRS
   receiver without hover

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
#include <firmware/debugger.hpp>
#include <firmware/effectors/quad_dshot.hpp>
#include <firmware/receivers/traditional.hpp>

static CRSFforArduino crsf_ = CRSFforArduino(&Serial2);

static hf::FlightController fc_;

static hf::TraditionalReceiver rxdata_;

static hf::QuadDshot motors_;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        rxdata_ = hf::TraditionalReceiver::Update(
                rxdata_,
                crsf_.readRcChannel(3),
                crsf_.readRcChannel(1),
                crsf_.readRcChannel(2),
                crsf_.readRcChannel(4),
                crsf_.readRcChannel(5),
                millis());
    }
}

void setup()
{
    // Start receiver
    if (!crsf_.begin()) {
        crsf_.end();
        hf::Debugger::ReportForever("Unable to start ELRS receiver");
    }
    crsf_.setRcChannelsCallback(onReceiveRcChannels);

    // Start flight control, no hoverdeck
    fc_.Begin(false);

    // Start motors
    motors_.Begin();
}

void loop()
{
    // This will trigger onReceiveRcChannels() above
    crsf_.update();

    // Run core algorithm to get setpoint from PID controllers
    const auto setpoint = fc_.Update(rxdata_, motors_.GetMotorValues(), 4);

    // Run the mixer and motors
    motors_.Run(fc_, setpoint);
}
