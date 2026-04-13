/*
   Hackflight for Teensy 4.0 with ELRs receiver

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
#include <dshot-teensy4.hpp>  

// Hackflight library

#include <hackflight.h>
#include <datatypes.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

#include <firmware/debugging.hpp>
#include <firmware/flying.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/led.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/profiling.hpp>
#include <firmware/safety.hpp>
#include <firmware/timer.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
using namespace hf;

static const uint8_t LED_PIN = 9;

// Rate constants
static constexpr float EKF_PREDICTION_RATE_HZ       = 100;
static constexpr float FLYING_CHECK_RATE_HZ         = 25;
static constexpr float FLOWDECK_ACQUISITION_RATE_HZ = 100;

// Devices
static IMU _imu;
static auto _led = LED(LED_PIN);
static ZRanger _zranger;
static OpticalFlowSensor _flowsensor;

// Timers
static auto _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
static auto _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
static auto _flowdeckTimer = Timer(FLOWDECK_ACQUISITION_RATE_HZ);

// Setup
void setup()
{
    _imu.begin();
    _led.begin(); 
    _zranger.begin();
    _flowsensor.begin();
}

// Loop
void loop()
{
    static Debugger _debugger;

    static EKF _ekf;
    static FlyingCheck _flyingCheck;
    static ImuFilter _imuFilter;
    static OpticalFlowFilter _opticalFlowFilter;
    static ZRangerFilter _zrangerFilter;

    _led.blink(_imuFilter.isGyroCalibrated);

    const auto imuraw = _imu.read();

    _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
            _imu.gyroRangeDps(), _imu.accelRangeGs());

    // Slower EKF update with range, optical flow
    if (_flowdeckTimer.ready()) {
        _zrangerFilter = ZRangerFilter::update(_zrangerFilter, _zranger.read());
        _opticalFlowFilter = OpticalFlowFilter::update(_opticalFlowFilter,
                micros(), _flowsensor.read());
        _ekf = EKF::update(_ekf, _zrangerFilter, _opticalFlowFilter);
    }

    // Run the system dynamics to predict the state forward.
    if (_ekfPredictionTimer.ready()) {
        _ekf = EKF::predict(_ekf, millis(), _flyingCheck.isFlying); 
    }

    // Faster EKF update with IMU readings
    _ekf = EKF::update(_ekf, _imuFilter.output, millis());

    _debugger.report(EKF::getVehicleState(_ekf), true);
}
