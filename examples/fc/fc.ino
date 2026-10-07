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

// Standard Arduino libraries
#include <SPI.h>
#include <Wire.h>

// Third-party libraries
#include <pmw3901.hpp>
#include <Adafruit_VL53L1X.h>

#include <hackflight.h>
#include <firmware/fc.hpp>
#include <firmware/debugger.hpp>
#include <firmware/drivers/error.hpp>
#include <firmware/hoverdeck.hpp>
#include <firmware/motors/quad_dshot.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/receiver.hpp>
#include <firmware/timer.hpp>

static constexpr uint8_t kVoltageInputPin = A9;
static const uint8_t kLedPin = 9;

static hf::Receiver rx_;

void serialEvent3()
{
    while (Serial3.available()) {
        rx_ = hf::Receiver::ParseByte(rx_, Serial3.read(), millis());
    }
}

static Adafruit_VL53L1X vl53l1x_;

static PMW3901 pmw3901_;

static hf::FlightController fc_;

static hf::QuadDshot motors_;

static void HoverDeckStart()
{
    (void)pmw3901_;

    Wire1.begin();
    Wire1.setClock(400000);
    delay(100);

    if (!vl53l1x_.begin(0x29, &Wire1)) {
        hf::Error::ReportForever("Unable to initialize VL53L1X");
    }

    if (!vl53l1x_.startRanging()) {
        hf::Error::ReportForever("VL53L1X failed to start ranging");
    }

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms
    vl53l1x_.setTimingBudget(50);
}

static auto HoverDeckRead() -> float
{
    static float distance_;

    if (vl53l1x_.dataReady())  {

        distance_ = vl53l1x_.distance();

        // Prepare for another reading
        vl53l1x_.clearInterrupt();
    }

    return distance_;
}

void setup()
{
    // Start receiver comms
    Serial3.begin(115200);

    // Wait a sec for the receiver to kick in
    delay(1000);

    // Enable heartbeat LED
    pinMode(kLedPin, OUTPUT); 

    // Start Z-ranger
    HoverDeckStart();

    // Start optical-flow sensor
    //OpticalFlowStart();

    // Start flight control
    fc_.Begin();

    // Start motors
    motors_.Begin();
}

void loop()
{
    // Receiver parses new data via serial event, so check arming here
    rx_ = hf::Receiver::CheckArming(rx_);

    // Run core algorithm to get setpoint from PID controllers and send
    // telemetry
    const auto setpoint = fc_.Update(
            micros(),
            HoverDeckRead(),
            analogRead(kVoltageInputPin),
            rx_,motors_.GetMotorValues(),
            4);

    const auto led_status = fc_.GetLedStatus();
    if (led_status == hf::FlightController::kLedOff) {
        digitalWrite(kLedPin, LOW);
    }
    else if (led_status == hf::FlightController::kLedOn) {
        digitalWrite(kLedPin, HIGH);
    }

    // Run the mixer and motors
    motors_.Run(setpoint, fc_.IsSafeToFly(), fc_.IsArmed());

    // Periodically send telemetry (receiver setpoint + vehicle state) to the
    // dongle
    if (fc_.IsTelemetryReady(millis())) {
        const auto telemetry_bytes = fc_.GetTelemetryBytes(rx_);
        Serial3.write(telemetry_bytes.bytes, telemetry_bytes.count);
    }
}
