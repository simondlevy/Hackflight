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
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

#include <firmware/debugging.hpp>
#include <firmware/flying.hpp>
#include <firmware/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/imu.hpp>
#include <firmware/led.hpp>
#include <firmware/opticalflow/filter.hpp>
#include <firmware/opticalflow/sensor.hpp>
#include <firmware/profiling.hpp>
#include <firmware/rx.hpp>
#include <firmware/safety.hpp>
#include <firmware/timer.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/zranger/filter.hpp>
#include <firmware/zranger/sensor.hpp>
using namespace hf;

#define _POSHOLD

#ifdef _POSHOLD
static const uint8_t LED_PIN = 9;
#else
static const uint8_t LED_PIN = LED_BUILTIN;
#endif

// Rate constants
static constexpr float EKF_PREDICTION_RATE_HZ       = 100;
static constexpr float FLYING_CHECK_RATE_HZ         = 25;
static constexpr float FLOWDECK_ACQUISITION_RATE_HZ = 100;

// Devices
static IMU _imu;
static auto _led = LED(LED_PIN);
static auto _rx = RX(&Serial5);
static auto _motors = DshotTeensy4({2, 3, 4, 5});
static ZRanger _zranger;
static OpticalFlowSensor _flowsensor;

// Timers
static auto _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
static auto _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
static auto _flowdeckTimer = Timer(FLOWDECK_ACQUISITION_RATE_HZ);

// Setup
void setup()
{
    _rx.begin();
    _imu.begin();
    _motors.begin(); 
    _led.begin(); 

#ifdef _POSHOLD
    _zranger.begin();
    _flowsensor.begin();
#else
    (void)_zranger;
    (void)_flowsensor;
#endif
}

// Loop
void loop()
{

    // Debugging
    static Debugger _debugger;
    static Profiler _profiler;

    // Computation
    static EKF _ekf;
    static FlyingCheck _flyingCheck;
    static ImuFilter _imuFilter;
    static Mixer _mixer;
    static StabilizerPid _stabilizerPid;
    static ZRangerFilter _zrangerFilter;

    // Flight mode
    static mode_e _mode;

    const auto dt = Timer::getDt();

    _led.blink(_imuFilter.isGyroCalibrated);

    // Disable arming while gyro is calibrating
    const auto rxdata =
        _imuFilter.isGyroCalibrated ? _rx.read() : RX::Data();

    const auto imuraw = _imu.read();

    _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
            _imu.gyroRangeDps(), _imu.accelRangeGs());

#ifdef _POSHOLD
    if (_flowdeckTimer.ready()) {
        _zrangerFilter = ZRangerFilter::step( _zrangerFilter, _zranger.read());
        _ekf.enqueueRange(_zrangerFilter);
        //const auto flowData = _flowSensor.read();
        //printf("%+03d %+03d %s\n", flowData.x, flowData.y, flowData.got_motion ? "yes" : "no");
    }
#else
    (void)_flowdeckTimer;
    (void)_zrangerFilter;
#endif

    _ekf.enqueueImu(_imuFilter.output);

    const uint32_t msec_curr = millis();

    if (_flyingCheckTimer.ready()) {
        _flyingCheck = FlyingCheck::run(
                _flyingCheck, millis(), _mixer.motorvals, 4);
    }

    // Run the system dynamics to predict the state forward.
    if (_ekfPredictionTimer.ready()) {
        _ekf.predict(msec_curr, _flyingCheck.isFlying); 
    }

    const auto state = _ekf.getVehicleState(msec_curr);

    _mode = Safety::updateMode(state, rxdata, _imuFilter, _mode);

    const auto setpoint = mksetpoint(rxdata.axes);

#ifdef _POSHOLD
    _debugger.report(state, true);
#else
    _debugger.report(state);
#endif
    //_profiler.report();

    _stabilizerPid = StabilizerPid::run( _stabilizerPid,
            !rxdata.is_throttle_down, dt, state, setpoint);

    _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

    if (_mode != MODE_PANIC) {
        _motors.run(rxdata.is_armed, _mixer.motorvals);
    }
}
