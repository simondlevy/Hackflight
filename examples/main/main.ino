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

// Standard Arduino libraries
#include <Wire.h> 

// Third-party libraries
#include <BMI088.h>
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

static const uint32_t FREQ_EKF_PREDICTION = 100;

static const uint32_t FREQ_FLYING_CHECK = 25;

static const uint8_t ZRANGER_INTERRUPT_PIN = 7;

static auto _rx = RX(&Serial5);

static auto _motors = DshotTeensy4({2, 3, 4, 5});

static auto _led = LED(13);

static ZRanger _zranger;

static StabilizerPid _stabilizerPid;

static Mixer _mixer;

static IMU _imu;

static ImuFilter _imuFilter;

static EKF _ekf;

static FlyingCheck _flyingCheck;

static mode_e _mode;

static Timer _ekfPredictionTimer;

static Timer _flyingCheckTimer;

void setup()
{
    _rx.begin();

    _imu.begin();

    _motors.begin(); 

    _led.begin(); 

    _zranger.begin(ZRANGER_INTERRUPT_PIN);
}

void loop()
{
    const auto dt = Timer::getDt();

    _led.blink(_imuFilter.isGyroCalibrated);

    // Disable arming while gyro is calibrating
    const auto rxdata =
        _imuFilter.isGyroCalibrated ? _rx.read() : RxData();

    const auto imuraw = _imu.read();

    _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
                IMU::gyroRangeDps(), IMU::accelRangeGs());

    _ekf.enqueueImu(_imuFilter.output);

    static bool _didResetEstimation;

    const uint32_t msec_curr = millis();

    if (_didResetEstimation) {
        _ekf.reset(msec_curr);
        _didResetEstimation = false;
    }

    if (_flyingCheckTimer.ready(FREQ_FLYING_CHECK)) {
        _flyingCheck = FlyingCheck::run(
                _flyingCheck, millis(), _mixer.motorvals, 4);
    }

    // Run the system dynamics to predict the state forward.
    if (_ekfPredictionTimer.ready(FREQ_EKF_PREDICTION)) {
        _ekf.predict(msec_curr, _flyingCheck.isFlying); 
    }

    const auto state = _ekf.getVehicleState(msec_curr);

    _mode = Safety::updateMode(state, rxdata, _imuFilter, _mode);

    const auto setpoint = mksetpoint(rxdata.axes);

    //Debugger::report(state);
    //Profiler::report();

    _stabilizerPid = StabilizerPid::run( _stabilizerPid,
            !rxdata.is_throttle_down, dt, state, setpoint);

    _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

    if (_mode != MODE_PANIC) {
        _motors.run(rxdata.is_armed, _mixer.motorvals);
    }
}
