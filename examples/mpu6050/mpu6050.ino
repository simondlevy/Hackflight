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
#include <dshot-teensy4.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/imus/mpu6050.hpp>
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/filters/imufilter.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/elrs.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <firmware/profiling.hpp>
#include <pidcontrol/stabilizer.hpp>

static hf::RX _rx;

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

static hf::LED _led = hf::LED(13);

static hf::StabilizerPid _stabilizerPid;

static hf::Mixer _mixer;

static hf::IMU _imu;

static hf::ImuFilter _imuFilter;

// static hf::MadgwickFilter _madgwick;
static hf::EKF _ekf;

void setup()
{
    _rx.begin();

    _imu.begin();

    _motors.arm(); 

    _led.begin(); 
}

void loop()
{
    const auto dt = hf::Timer::getDt();

    _rx.read();

    const auto imuraw = _imu.read();

    _imuFilter.step(
            millis(), imuraw, _imu.gyroRangeDps(), _imu.accelRangeGs());

    _led.blink(millis(), _imuFilter.wasGyroBiasFound ? 1 : 3);

    _ekf.enqueueImu(_imuFilter.output);

    const bool isFlying = true;

    const auto state = _ekf.getVehicleState(millis(), isFlying);

    const auto setpoint = hf::mksetpoint(_rx.data.axes);

    _stabilizerPid = hf::StabilizerPid::run( _stabilizerPid,
            !_rx.data.is_throttle_down, dt, state, setpoint);

    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(_rx.data.is_armed, _mixer.motorvals);
}
