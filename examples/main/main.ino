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
#include <firmware/device/bmi088.hpp>
#include <firmware/device/debugging.hpp>
#include <firmware/device/elrs.hpp>
#include <firmware/device/led.hpp>
#include <firmware/device/profiling.hpp>
#include <firmware/device/timer.hpp>
#include <firmware/device/vl53l1x/vl53l1x.h>
#include <firmware/flying.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/rxdata.hpp>
#include <firmware/safety.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/zranger_filter.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>
using namespace hf;

// Constants
static constexpr float EKF_PREDICTION_RATE_HZ      = 100;
static constexpr float FLYING_CHECK_RATE_HZ        = 25;
static constexpr float ZRANGER_ACQUISITION_RATE_HZ = 100;
static constexpr uint8_t ZRANGER_INTERRUPT_PIN = 7;

// Devices
static IMU _imu;
static auto _led = LED(13);
static auto _rx = RX(&Serial5);
static auto _motors = DshotTeensy4({2, 3, 4, 5});
static auto _zranger = ZRanger(ZRANGER_INTERRUPT_PIN);

// Timers
static auto _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
static auto _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);
static auto _zrangerTimer = Timer(ZRANGER_ACQUISITION_RATE_HZ);

// Setup
void setup()
{
    _rx.begin();
    _imu.begin();
    _motors.begin(); 
    _led.begin(); 
    _zranger.begin();
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

    // Wait til gyro is calibrated before acquriing Z-Ranger data
    if (_imuFilter.isGyroCalibrated && _zrangerTimer.ready()) {
        _zrangerFilter = ZRangerFilter::step( _zrangerFilter, _zranger.read());
        _ekf.enqueueRange(_zrangerFilter);
    }

    const auto dt = Timer::getDt();

    _led.blink(_imuFilter.isGyroCalibrated);

    // Disable arming while gyro is calibrating
    const auto rxdata =
        _imuFilter.isGyroCalibrated ? _rx.read() : RxData();

    const auto imuraw = _imu.read();

    _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
            IMU::gyroRangeDps(), IMU::accelRangeGs());

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

    _debugger.report(state, true);
    //_profiler.report();

    _stabilizerPid = StabilizerPid::run( _stabilizerPid,
            !rxdata.is_throttle_down, dt, state, setpoint);

    _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

    if (_mode != MODE_PANIC) {
        _motors.run(rxdata.is_armed, _mixer.motorvals);
    }
}
