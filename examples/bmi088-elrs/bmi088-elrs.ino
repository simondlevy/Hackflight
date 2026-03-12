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
#include <firmware/datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/estimators/ekf/ekf.hpp>
#include <firmware/imu/new/filter.hpp>
#include <firmware/imu/bmi088.hpp>
#include <firmware/imu/new/filter.hpp>
#include <firmware/led.hpp>
#include <firmware/rx/elrs.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

static hf::RX _rx;

static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

static hf::LED _led = hf::LED(13);

static hf::StabilizerPid _stabilizerPid;

static hf::Mixer _mixer;

static hf::IMU _imu;

////////////////////////////////////////////////////////////////////////

static const uint32_t FREQ_EKF_PREDICTION = 100;

static hf::ImuFilter _imuFilter;

static hf::EKF _ekf;

static auto getVehicleState(const hf::ImuRaw & imuraw, 
        const bool isFlying) -> hf::VehicleState
{
    _imuFilter.step(millis(), imuraw);

    const auto imuIsCalibrated = _imuFilter.wasGyroBiasFound;
    (void)imuIsCalibrated; // XXX should rapid-blink LED until IMU calibrated

    _ekf.enqueueImu(_imuFilter.output);

    // Get state estimate from EKF
    const auto state = _ekf.getStateEstimate(millis(), isFlying);

    // Get angular velocities directly from gyro
    return hf::VehicleState(
            state.dx,
            state.dy,
            state.z,
            state.dz,
            state.phi,
            _imuFilter.output.gyroDps.x,
            state.theta,
            _imuFilter.output.gyroDps.y,
            state.psi,
            -_imuFilter.output.gyroDps.z); // negate for nose-right positive.y
}

////////////////////////////////////////////////////////////////////////

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

    _led.blink(); 

    _rx.read();

    const bool isFlying = true; // XXX

    const auto imuraw = _imu.read();

    const auto state = getVehicleState(imuraw, isFlying);

    const auto setpoint = hf::mksetpoint(_rx.chanvals);

    hf::Debugger::debug(state);
    //hf::Debugger::profile();

    _stabilizerPid = hf::StabilizerPid::run( _stabilizerPid,
            !_rx.is_throttle_down, dt, state, setpoint);

    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(_rx.is_armed, _mixer.motorvals);
}
