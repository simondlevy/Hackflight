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

// C++ STL
#include <vector>

// Standard Arduino libraries
#include <SPI.h>
#include <Wire.h>

// Third-party libraries
#include <Adafruit_VL53L1X.h>
#include <BMI088.h>
#include <dshot-teensy4.hpp>  
#include <pmw3901.hpp>

// Hackflight library
#include <hackflight.h>
#include <firmware/debugger.hpp>
#include <firmware/flight_control.hpp>
#include <firmware/imu/rawdata.hpp>
#include <firmware/led.hpp>
#include <firmware/optical_flow.hpp>
#include <mixers/quadx.hpp>

static constexpr uint8_t kVoltageInputPin = A9;
static const uint8_t kLedPin = 9;

static hf::FlightController fc_;

// Receiver ------------------------------------------------------------------

void serialEvent3()
{
    while (Serial3.available()) {
        fc_ = hf::FlightController::ParseReceiverByte(
                fc_, Serial3.read(), millis());
    }
}

// ZRanger -------------------------------------------------------------------

static Adafruit_VL53L1X vl53l1x_;

static void ZRangerStart()
{
    Wire1.begin();
    Wire1.setClock(400000);
    delay(100);

    if (!vl53l1x_.begin(0x29, &Wire1)) {
        hf::Debugger::ReportForever("Unable to initialize VL53L1X");
    }

    if (!vl53l1x_.startRanging()) {
        hf::Debugger::ReportForever("VL53L1X failed to start ranging");
    }

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms
    vl53l1x_.setTimingBudget(50);
}

static auto ZRangerRead() -> float
{
    static float distance_;

    if (vl53l1x_.dataReady())  {

        distance_ = vl53l1x_.distance();

        // Prepare for another reading
        vl53l1x_.clearInterrupt();
    }

    return distance_;
}

// Optical flow --------------------------------------------------------------

static PMW3901 pmw3901_;

static void OpticalFlowStart()
{
    SPI.begin();

    if (!pmw3901_.begin()) {
        hf::Debugger::ReportForever("Unable to initialize PMW3901");
    }
}

static auto OpticalFlowRead() -> hf::OpticalFlowData
{
    int16_t dx = 0;
    int16_t dy = 0;
    auto moved = false; // we ignore this

    pmw3901_.readMotion(dx, dy, moved);

    return hf::OpticalFlowData(dx, dy);
}

// Imu -----------------------------------------------------------------------

static constexpr Bmi088Gyro::Range kGyroRange = Bmi088Gyro::RANGE_2000DPS;
static constexpr Bmi088Accel::Range kAccelRange = Bmi088Accel::RANGE_24G;

// The SDO pin should either be pulled low for the 0x18/0x68
// addresses, high for 0x19/0x69
static Bmi088Accel accel_ = Bmi088Accel(Wire, 0x18);
static Bmi088Gyro gyro_ = Bmi088Gyro(Wire, 0x68);

static auto ImuOkay(const int status) -> bool
{
    return status >= 0;
}

static void ImuStart()
{
    if (!(ImuOkay(gyro_.begin()) &&

        ImuOkay(accel_.begin()) &&

        ImuOkay(gyro_.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&

        ImuOkay(gyro_.setRange(kGyroRange)) &&

        ImuOkay(gyro_.pinModeInt3(
                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

        ImuOkay(gyro_.mapDrdyInt3(true)) &&

        ImuOkay(accel_.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&

        ImuOkay(accel_.setRange(kAccelRange)))) {

        hf::Debugger::ReportForever("Unable to start Imu");
    }
}

static auto ImuRead() -> hf::Imu::RawData
{

    static int16_t gx_;
    static int16_t gy_;
    static int16_t gz_;
    static constexpr int16_t granges[5] = {2000, 1000, 500, 250, 125};

    static int16_t ax_;
    static int16_t ay_;
    static int16_t az_;
    static constexpr int16_t aranges[4] = {3, 6, 12, 24};

    if (gyro_.getDrdyStatus()) {

        gyro_.readSensor();
        gx_ = gyro_.getGyroX_raw();
        gy_ = gyro_.getGyroY_raw();
        gz_ = gyro_.getGyroZ_raw();

        accel_.readSensor();
        ax_ = accel_.getAccelX_raw();
        ay_ = accel_.getAccelY_raw();
        az_ = accel_.getAccelZ_raw();
    }

    return hf::Imu::RawData(
            hf::Imu::ThreeAxisRaw(gx_, gy_, gz_, granges[kGyroRange]),
            hf::Imu::ThreeAxisRaw(ax_, ay_, az_, aranges[kAccelRange]));
}

// Motors --------------------------------------------------------------------

static hf::QuadXMixer mixer_;

static std::vector<uint8_t> kMotorPins = {4, 5, 2, 3};

DshotTeensy4 motors_ = DshotTeensy4(kMotorPins);

static void MotorsStart()
{
    motors_.begin();
}

static auto MotorsGetValues() -> std::vector<float>
{
    return hf::QuadXMixer::NewGetMotorValues(mixer_);
}

static void MotorsRun(const hf::Setpoint & setpoint)
{
    mixer_ = hf::QuadXMixer::Run(setpoint);

    // Run motors if safe
    if (hf::FlightController::IsSafeToFly(fc_)) {
        MotorsGetValues();
        motors_.run(hf::FlightController::IsArmed(fc_), MotorsGetValues());
    }
}

// Profiling -----------------------------------------------------------------

static void ProfilerRun(const bool should_run=true)
{
    if (should_run) {
        static uint32_t count_;
        static uint32_t msec_;
        const auto msec = millis();
        if (msec - msec_ > 1000) {
            if (count_ > 0) {
                printf("%d\n", (int)count_);
            }
            count_ = 0;
            msec_ = msec;
        }
        count_++;
    }
}

// Main ----------------------------------------------------------------------

void setup()
{
    // Start receiver comms
    Serial3.begin(115200);

    // Wait a sec for the receiver to kick in
    delay(1000);

    // Enable heartbeat LED
    pinMode(kLedPin, OUTPUT); 

    // Start the sensors
    ImuStart();
    ZRangerStart();
    OpticalFlowStart();

    // Start motors
    MotorsStart();
}

void loop()
{
    // We should reach aprrox. 1600 Hz in this loop
    ProfilerRun();

    // Update core algorithm with all inputs and run PID controller
    fc_ = hf::FlightController::Update(fc_, micros(), ImuRead(),
            analogRead(kVoltageInputPin), MotorsGetValues(),
            hf::PidController::kControlStabilize);

    // Get final setpoint from PID controller
    const auto setpoint = hf::FlightController::GetSetpoint(fc_);

    // Run sensor fusion on hover-deck as indicated
    fc_ = hf::FlightController::ShouldUpdateHover(fc_) ?
        hf::FlightController::UpdateHover(fc_, micros(), ZRangerRead(),
                OpticalFlowRead()) :
        fc_;

    // Blink LED to indicate status
    const auto led_status = hf::FlightController::GetLedStatus(fc_);
    if (led_status == hf::Led::kStatusOff) {
        digitalWrite(kLedPin, LOW);
    }
    else if (led_status == hf::Led::kStatusOn) {
        digitalWrite(kLedPin, HIGH);
    }

    // Run the mixer and motors
    MotorsRun(setpoint);

    // Periodically send telemetry (receiver setpoint + vehicle state) to the
    // dongle
    if (hf::FlightController::ShouldSendTelemetry(fc_)) {
        const auto telemetry_bytes =
            hf::FlightController::GetTelemetryBytes(fc_);
        Serial3.write(telemetry_bytes.data(), telemetry_bytes.size());
    }
}
