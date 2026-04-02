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
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/bmi088.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/led.hpp>
#include <firmware/elrs.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>
using namespace hf;

static RX _rx;

static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

static LED _led = LED(13);

static StabilizerPid _stabilizerPid;

static Mixer _mixer;

static IMU _imu;

////////////////////////////////////////////////////////////////////////

static const uint32_t FREQ_EKF_PREDICTION = 100;

static ImuFilter _imuFilter;

static EKF _ekf;

static auto getVehicleState(const ImuRaw & imuraw, 
        const bool isFlying) -> VehicleState
{
    _imuFilter.step(millis(), imuraw);

    _ekf.enqueueImu(_imuFilter.output);

    static Timer _timer;

    static bool _didResetEstimation;

    const uint32_t msec_curr = millis();

    if (_didResetEstimation) {
        _ekf.reset(msec_curr);
        _didResetEstimation = false;
    }

    // Run the system dynamics to predict the state forward.
    if (_timer.ready(FREQ_EKF_PREDICTION)) {
        _ekf.predict(msec_curr, isFlying); 
    }

    // Get state estimate from EKF
    const auto state = _ekf.getStateEstimate(msec_curr);

    // Get angular velocities directly from gyro
    return VehicleState(
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

    _motors.begin(); 

    _led.begin(); 
}

void loop()
{
    const auto dt = Timer::getDt();

    _led.blink(_imuFilter.wasGyroBiasFound);

    _rx.read();

    const bool isFlying = true; // XXX

    const auto imuraw = _imu.read();

    const auto state = getVehicleState(imuraw, isFlying);

    const auto setpoint = mksetpoint(_rx.chanvals);

    //Debugger::report(state);
    //Profiler::report();

    _stabilizerPid = StabilizerPid::run( _stabilizerPid,
            !_rx.is_throttle_down, dt, state, setpoint);

    _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(_rx.is_armed, _mixer.motorvals);
}
