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
#include <dshot-teensy4.hpp>  

// Hackflight library

#include <hackflight.h>
#include <datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/flying.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/led.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/profiling.hpp>
#include <firmware/receiver.hpp>
#include <firmware/safety.hpp>
#include <firmware/timer.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>
using namespace hf;

//#define POSHOLD

#ifdef POSHOLD
static const uint8_t LED_PIN = 9;
#else
static const uint8_t LED_PIN = LED_BUILTIN;
#endif

// Rate constants
static constexpr float EKF_PREDICTION_RATE_HZ       = 100;
static constexpr float FLYING_CHECK_RATE_HZ         = 25;
static constexpr float FLOWDECK_ACQUISITION_RATE_HZ = 100;
static constexpr float TELEMETRY_RATE_HZ = 50;

// Devices
static IMU _imu;
static auto _led = LED(LED_PIN);
static auto _motors = DshotTeensy4({2, 3, 4, 5});
#ifdef POSHOLD
static ZRanger _zranger;
static OpticalFlowSensor _flowsensor;
#endif

// Timers
static auto _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
static auto _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
static auto _flowdeckTimer = Timer(FLOWDECK_ACQUISITION_RATE_HZ);
static auto _telemetryTimer = Timer(TELEMETRY_RATE_HZ);

// Debugging / profiling
static Debugger _debugger;
static Profiler _profiler;

// Setup
void setup()
{
    Receiver::begin();

    _imu.begin();
    _motors.begin(); 
    _led.begin(); 

#ifdef POSHOLD
    _zranger.begin();
    _flowsensor.begin();
#endif
}

// Loop
void loop()
{
    // Computation
    static EKF _ekf;
    static FlyingCheck _flyingCheck;
    static ImuFilter _imuFilter;
    static Mixer _mixer;
    static StabilizerPid _stabilizerPid;
#ifdef POSHOLD
    static OpticalFlowFilter _opticalFlowFilter;
    static ZRangerFilter _zrangerFilter;
#endif

    // Flight mode
    static mode_e _mode;

    const auto dt = Timer::getDt();

    _led.blink(_imuFilter.isGyroCalibrated);

    auto rxdata = Receiver::read();

    // Disable arming while gyro is calibrating
    rxdata = _imuFilter.isGyroCalibrated ? rxdata : Receiver::Data();

    const auto imuraw = _imu.read();

    _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
            _imu.gyroRangeDps(), _imu.accelRangeGs());

#ifdef POSHOLD
    // Slower EKF update with range, optical flow
    if (_flowdeckTimer.ready()) {
        _zrangerFilter = ZRangerFilter::update(_zrangerFilter, _zranger.read());
        _opticalFlowFilter = OpticalFlowFilter::update(_opticalFlowFilter,
                micros(), _flowsensor.read());
        _ekf = EKF::update(_ekf, _zrangerFilter, _opticalFlowFilter);
    }
#endif

    // Run the system dynamics to predict the state forward.
    if (_ekfPredictionTimer.ready()) {
        _ekf = EKF::predict(_ekf, millis(), _flyingCheck.isFlying); 
    }

    // Faster EKF update with IMU readings
    _ekf = EKF::update(_ekf, _imuFilter.output, millis());

    // Get vehicle state from EKF
    const auto state = EKF::getVehicleState(_ekf);

    // Send telemetry periodically
    if (_telemetryTimer.ready()) {
        Receiver::send(state);
    }

    // Check receiver timeout
    rxdata = Receiver::Data::checkTimeout(rxdata, millis());

    //_debugger.report(rxdata);
    //_profiler.report();

    _mode = Safety::updateMode(state, rxdata, _imuFilter, _mode);

    const auto setpoint = mksetpoint(rxdata.axes);
    _stabilizerPid = StabilizerPid::run( _stabilizerPid,
            !rxdata.is_throttle_down, dt, state, setpoint);

    _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

    if (_mode != MODE_PANIC) {
        _motors.run(rxdata.is_armed, _mixer.motorvals);
    }

    if (_flyingCheckTimer.ready()) {
        _flyingCheck = FlyingCheck::run(
                _flyingCheck, millis(), _mixer.motorvals, 4);
    }
}
