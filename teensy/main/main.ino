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
#include <CRSFforArduino.hpp>

// Hackflight library

#include <hackflight.h>
#include <datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/flying.hpp>
#include <firmware/ekf/ekf.hpp>
#include <firmware/imu/filter.hpp>
#include <firmware/imu/sensor.hpp>
#include <firmware/led.hpp>
#include <firmware/profiling.hpp>
#include <firmware/rx.hpp>
#include <firmware/safety.hpp>
#include <firmware/timer.hpp>
#include <firmware/setpoint.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>
using namespace hf;

static uint32_t _last_rx_msec;

static CRSFforArduino _crsf;

static RX _rx;

float scalechan(const uint8_t k)
{
    return RX::scale(_crsf.readRcChannel(k));
}

static void onReceiveRcChannels(
        serialReceiverLayer::rcChannels_t *rcChannels, void * obj)
{
    if (!rcChannels->failsafe) {

        RX::update(
                _rx, 
                _crsf.readRcChannel(3),
                _crsf.readRcChannel(1),
                _crsf.readRcChannel(2),
                _crsf.readRcChannel(4),
                _crsf.readRcChannel(5),
                millis());

        _rx.axes.thrust = scalechan(3);
        _rx.axes.roll = scalechan(1);
        _rx.axes.pitch = scalechan(2);
        _rx.axes.yaw = scalechan(4);

        _rx.aux = _crsf.readRcChannel(5);

        _last_rx_msec = millis();
    }
}

static auto rxread() -> RX
{
    _crsf.update();

    _rx.is_throttle_down = _rx.axes.thrust < RX::THROTTLE_DOWN_MAX;

    const auto msec_curr = millis();

    // Check failsafe via RX timeout
    if (_last_rx_msec > 0 &&
            msec_curr > _last_rx_msec &&
            msec_curr - _last_rx_msec > RX::TIMEOUT_MSEC) {
        _rx.is_armed = false;
    }

    // Push-button arming
    static float _chan5_prev;
    const auto chan5_curr = _rx.aux;
    if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
        _rx.is_armed =
            _rx.is_armed ? false :
            _rx.is_throttle_down ? true :
            _rx.is_armed;
    }
    _chan5_prev = chan5_curr;

    return _rx;
}    

static const uint8_t LED_PIN = LED_BUILTIN;

// Rate constants
static constexpr float EKF_PREDICTION_RATE_HZ       = 100;
static constexpr float FLYING_CHECK_RATE_HZ         = 25;
static constexpr float FLOWDECK_ACQUISITION_RATE_HZ = 100;

// Devices
static IMU _imu;
static auto _led = LED(LED_PIN);
static auto _motors = DshotTeensy4({2, 3, 4, 5});

// Timers
static auto _ekfPredictionTimer = Timer(EKF_PREDICTION_RATE_HZ);
static auto _flyingCheckTimer = Timer(FLYING_CHECK_RATE_HZ);

// Debugging
static Debugger _debugger;

// Setup
void setup()
{
    _crsf = CRSFforArduino(&Serial5);
    if (!_crsf.begin()) {
        _crsf.end();
        while (true) {
            printf("CRSF for Arduino initialisation failed!\n");
            delay(500);
        }
    }
    _crsf.setRcChannelsCallback(onReceiveRcChannels, nullptr);

    _imu.begin();
    _motors.begin(); 
    _led.begin(); 
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

    // Flight mode
    static mode_e _mode;

    const auto dt = Timer::getDt();

    _led.blink(_imuFilter.isGyroCalibrated);

    // Disable arming while gyro is calibrating
    const auto rx = _imuFilter.isGyroCalibrated ? rxread() : RX();

    _debugger.report(rx);

    const auto imuraw = _imu.read();

    _imuFilter = ImuFilter::step(_imuFilter, millis(), imuraw,
            _imu.gyroRangeDps(), _imu.accelRangeGs());

    // Run the system dynamics to predict the state forward.
    if (_ekfPredictionTimer.ready()) {
        _ekf = EKF::predict(_ekf, millis(), _flyingCheck.isFlying); 
    }

    // Faster EKF update with IMU readings
    _ekf = EKF::update(_ekf, _imuFilter.output, millis());

    const auto state = EKF::getVehicleState(_ekf);

    _mode = Safety::updateMode(state, rx, _imuFilter, _mode);

    const auto setpoint = mksetpoint(rx.axes);
    _stabilizerPid = StabilizerPid::run( _stabilizerPid,
            !rx.is_throttle_down, dt, state, setpoint);

    _mixer = Mixer::run(_mixer, _stabilizerPid.setpoint);

    if (_mode != MODE_PANIC) {
        _motors.run(rx.is_armed, _mixer.motorvals);
    }

    if (_flyingCheckTimer.ready()) {
        _flyingCheck = FlyingCheck::run(
                _flyingCheck, millis(), _mixer.motorvals, 4);
    }
}
