/*
   Hackflight main sketch

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
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/filters/imufilter.hpp>
#include <firmware/flying.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/elrs.hpp>
#include <firmware/safety.hpp>
//#include <firmware/sensors/imus/bmi088.hpp>
//#include <firmware/sensors/imus/lsm6dso_rot90ccw.hpp>
#include <firmware/sensors/imus/mpu6050.hpp>
#include <firmware/sensors/zranger.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <firmware/profiling.hpp>
#include <pidcontrol/stabilizer.hpp>

/* Update rates
FLOW/ZRANGER: 50 Hz
MAIN_LOOP = 1000
PIDS = 500;
EKF_PREDICTION = 100
*/

static hf::RX _rx;

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});
// static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

static hf::LED _led = hf::LED(13);

static hf::StabilizerPid _stabilizerPid;

static hf::Mixer _mixer;

static hf::IMU _imu;

static hf::ZRanger _zranger;

static hf::ImuFilter _imuFilter;

static hf::EKF _ekf;

static hf::FlyingCheck _flyingCheck;

static hf::mode_e _mode;

void setup()
{
    _rx.begin();

    _imu.begin();

    //_zranger.begin();

    _motors.begin(); 

    _led.begin(); 
}

void loop()
{
    const auto loop_start_usec = micros();

    /*
    if (_zranger.available()) {

        _zranger.read();
    }*/

    if (_imu.available()) {

        const auto imuraw = _imu.read();

        const auto dt = hf::Timer::getDt();

        const auto rxdata = _rx.read();

        _imuFilter = hf::ImuFilter::step(_imuFilter, millis(), imuraw,
                _imu.gyroRangeDps(), _imu.accelRangeGs());

        _led.blink(millis(), _imuFilter.isGyroCalibrated);

        _flyingCheck = _flyingCheck.run(
                _flyingCheck, millis(), _mixer.motorvals, 4);

        _ekf.enqueueImu(_imuFilter.output);

        const auto state = _ekf.getVehicleState(millis(), _flyingCheck.isFlying);

        _mode = hf::Safety::updateMode(state, rxdata, _imuFilter, _mode);

        const auto setpoint = hf::mksetpoint(rxdata.axes);

        _stabilizerPid = hf::StabilizerPid::run( _stabilizerPid,
                !rxdata.is_throttle_down, dt, state, setpoint);

        _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

        if (_mode != hf::MODE_PANIC) {
            _motors.run(rxdata.is_armed, _mixer.motorvals);
        }

        // hf::Debugger::report(state);
    }

    hf::Timer::runDelayLoop(loop_start_usec);

    //hf::Profiler::report();
}
