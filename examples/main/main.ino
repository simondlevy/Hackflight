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
#include <Adafruit_VL53L1X.h>
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
//#include <firmware/imus/bmi088.hpp>
#include <firmware/imus/lsm6dso_rot90ccw.hpp>
//#include <firmware/imus/mpu6050.hpp>
#include <firmware/setpoint.hpp>
#include <firmware/timer.hpp>
#include <mixers/bfquadx.hpp>
#include <firmware/profiling.hpp>
#include <pidcontrol/stabilizer.hpp>

/* Update rates
   MAIN_LOOP = 1000
   PIDS = 500;
 */

// ZRanger -------------------------------------------------------------------

static const uint8_t ZRANGER_INTERRUPT_PIN = 7;

//static constexpr float ZRANGER_UPDATE_RATE_HZ = 50;

static Adafruit_VL53L1X _zranger;

static volatile bool _zranger_is_data_ready;

static void zranger_handle_data_ready() {

    _zranger_is_data_ready = true;

    _zranger.clearInterrupt();
}

static void zranger_begin()
{
    Wire1.begin();
    Wire1.setClock(400000);

    if (!_zranger.begin(0x29, &Wire1)) {
        hf::Debugger::reportForever("Unable to initialize sensor");
    }

    if (!_zranger.startRanging()) {
        hf::Debugger::reportForever("Unable to start ranging");
    }

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    _zranger.setTimingBudget(50);

    // Polarity=1 => RISING
    _zranger.VL53L1X_SetInterruptPolarity(1);
    attachInterrupt(digitalPinToInterrupt(ZRANGER_INTERRUPT_PIN),
            zranger_handle_data_ready, RISING);

    // Clear interrupt to get things started
    _zranger.clearInterrupt();
}

static int16_t zranger_acquire()
{
    static int16_t distance;

    if (_zranger_is_data_ready) {
        distance = _zranger.distance();
        _zranger_is_data_ready = false;
    }

    return distance;
}

// ---------------------------------------------------------------------------

static constexpr float EKF_PREDICTION_RATE_HZ = 100;

static hf::RX _rx;

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});
// static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

static hf::LED _led = hf::LED(13);
static hf::StabilizerPid _stabilizerPid;
static hf::Mixer _mixer;
static hf::IMU _imu;
static hf::ImuFilter _imuFilter;
static hf::EKF _ekf;
static hf::FlyingCheck _flyingCheck;
static hf::mode_e _mode;
static hf::Timer _ekfPredictionTimer;
static hf::Timer _zrangerTimer;

void setup()
{
    _rx.begin();

    _imu.begin();

    zranger_begin();

    _motors.begin(); 

    _led.begin(); 
}

void loop()
{
    const auto loop_start_usec = micros();

    const auto zranger_distance = zranger_acquire();
    (void)zranger_distance;

    if (_ekfPredictionTimer.ready(micros())) {
        _ekf = hf::EKF::predict(_ekf, millis(), _flyingCheck.isFlying);
    }

    if (_imu.available()) {

        const auto imuraw = _imu.read();

        const auto dt = hf::Timer::getDt();

        const auto rxdata = _rx.read();

        _imuFilter = hf::ImuFilter::step(_imuFilter, millis(), imuraw,
                _imu.gyroRangeDps(), _imu.accelRangeGs());

        _led.blink(millis(), _imuFilter.isGyroCalibrated);

        _flyingCheck = _flyingCheck.run(
                _flyingCheck, millis(), _mixer.motorvals, 4);

        _ekf = hf::EKF::update(_ekf, millis(), _imuFilter.output);

        const auto state = hf::EKF::getVehicleState(_ekf, _imuFilter.output);

        _mode = hf::Safety::updateMode(state, rxdata, _imuFilter, _mode);

        const auto setpoint = hf::mksetpoint(rxdata.axes);

        _stabilizerPid = hf::StabilizerPid::run( _stabilizerPid,
                !rxdata.is_throttle_down, dt, state, setpoint);

        _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

        if (_mode != hf::MODE_PANIC) {
            _motors.run(rxdata.is_armed, _mixer.motorvals);
        }

        //hf::Debugger::report(state);
    }

    hf::Timer::runDelayLoop(loop_start_usec);

    hf::Profiler::report();
}
