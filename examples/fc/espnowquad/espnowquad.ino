/*
   Hackflight flight-controller sketch for Teensy quadcopter using ESP32
   receiver with MSP protocol

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

#include <hackflight.h>
#include <firmware/fc.hpp>
#include <firmware/debugger.hpp>
#include <firmware/effectors/quad_dshot.hpp>
#include <firmware/receivers/espnow.hpp>

#include <firmware/timer.hpp>

static hf::Debugger debugger_;

static hf::Timer timer_ = hf::Timer(50);

static hf::EspNowReceiver rx_;

void serialEvent3()
{
    while (Serial3.available()) {
        rx_ = hf::EspNowReceiver::ParseByte(rx_, Serial3.read(), millis());
    }
}

static hf::FlightController fc_;

static hf::QuadDshot motors_;

static void SendTelemetry()
{
    if (timer_.Ready()) {

        const float msg[15] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 39, 41, 43};

        //Serial3.write((uint8_t *)msg, sizeof(msg));
    }
}

void setup()
{
    // Start receiver comms
    Serial3.begin(115200);

    // Wait a sec for the receiver to kick in
    delay(1000);

    // Start flight control, no hoverdeck
    //fc_.Begin(false);

    // Start motors
    //motors_.Begin();
}

void loop()
{
    // Receiver parses new data via serial event, so check arming here
    rx_ = hf::EspNowReceiver::CheckArming(rx_);

    debugger_.Report(rx_);

    SendTelemetry();

    // Run core algorithm to get setpoint from PID controllers
    //const auto setpoint = fc_.Update(rx_, motors_.GetMotorValues(), 4);

    // Run the mixer and motors
    //motors_.Run(fc_, setpoint);

    // Send telemetry to base station over Serial3
    //fc_.SendTelemetry(Serial3, setpoint);
}
