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
#include <CRSFforArduino.hpp>

// Hackflight library
#include <hackflight.h>
#include <datatypes.hpp>
#include <firmware/debugging.hpp>
#include <firmware/estimators/ekf/ekf.hpp>
#include <firmware/imu/imu.hpp>
#include <firmware/led.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/pids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

// IMU ------------------------------------------------------------

static const int16_t GYRO_SCALE = 2000;
static const int16_t ACCEL_SCALE = 24;

static hf::IMU _imu;

static Bmi088Accel accel(Wire, 0x19);
static Bmi088Gyro gyro(Wire, 0x69);

static bool okay(const int status)
{
    return status >= 0;
}

static bool imu_device_init()
{
    return 

        okay(gyro.begin()) &&

        okay(accel.begin()) &&

        okay(gyro.setOdr(Bmi088Gyro::ODR_1000HZ_BW_116HZ)) &&

        okay(gyro.setRange(Bmi088Gyro::RANGE_2000DPS)) &&

        okay(gyro.pinModeInt3(
                    Bmi088Gyro::PIN_MODE_PUSH_PULL,
                    Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH)) &&

        okay(gyro.mapDrdyInt3(true)) &&

        okay(accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_145HZ)) &&

        okay(accel.setRange(Bmi088Accel::RANGE_24G));
}

static void imu_device_read(
        int16_t & gx, int16_t & gy, int16_t & gz,
        int16_t & ax, int16_t & ay, int16_t & az)
{
    gyro.readSensor();

    gx = gyro.getGyroX_raw();
    gy = gyro.getGyroY_raw();
    gz = gyro.getGyroZ_raw();

    accel.readSensor();

    ax = accel.getAccelX_raw();
    ay = accel.getAccelY_raw();
    az = accel.getAccelZ_raw();
}
// EKF ------------------------------------------------------------

static const uint32_t FREQ_EKF_PREDICTION = 100;

static hf::EKF _ekf;

// Receiver -------------------------------------------------------

static constexpr uint32_t RX_TIMEOUT_USEC = 500'000;

static CRSFforArduino _crsf;

static float _rx_chanvals[5];

static constexpr int CHANNEL_COUNT = 5;

static auto scalechan(serialReceiverLayer::rcChannels_t *rcChannels,
        const int k) -> float
{
    return map((float)_crsf.readRcChannel(k), 989, 2012, -1, +1);
}

static uint32_t _last_rx_usec;

static void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (!rcChannels->failsafe) {

        _rx_chanvals[0] = scalechan(rcChannels, 3);
        _rx_chanvals[1] = scalechan(rcChannels, 1);
        _rx_chanvals[2] = scalechan(rcChannels, 2);
        _rx_chanvals[3] = scalechan(rcChannels, 4);
        _rx_chanvals[4] = scalechan(rcChannels, 5);

        _last_rx_usec = micros();
    }
}

static void rx_init()
{
    if (!_crsf.begin()) {

        _crsf.end();

        while (true) {
            printf("CRSF for Arduino initialisation failed!\n");
            delay(500);
        }
    }

    _crsf.setRcChannelsCallback(onReceiveRcChannels);
}

// Motors ---------------------------------------------------------

static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

// LED -------------------------------------------------------------

static hf::LED _led = hf::LED(13);

// Safety ----------------------------------------------------------

static const float THROTTLE_DOWN_MAX = -0.95;

// Helper functions ------------------------------------------------

static auto getVehicleState(const bool isFlying, const hf::Vec3 & gyroDps)
    -> hf::VehicleState
{
    static hf::Timer _timer;

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
    const hf::EstimatedState state = _ekf.getStateEstimate(msec_curr);

    // Get angular velocities directly from gyro
    return hf::VehicleState(
            state.dx,
            state.dy,
            state.z,
            state.dz,
            state.phi,
            gyroDps.x,
            state.theta,
            gyroDps.y,
            state.psi,
            -gyroDps.z); // negate for nose-right positive.y
}

static float getDt(const uint32_t usec_curr)
{
    static uint32_t _usec_prev;
    const float dt = (usec_curr - _usec_prev)/1000000.0;
    _usec_prev = usec_curr;

    return dt;
}

// Main ----------------------------------------------------------------------

void setup()
{
    Serial.begin(0); 

    rx_init();

    imu_device_init();

    _motors.arm(); 

    _led.blinkOnStartup(); 
}

void loop()
{
    hf::Debugger::profile();

    const auto usec_curr = micros();      

    const auto dt = getDt(usec_curr);

    _led.blinkInLoop(usec_curr); 

    _crsf.update();

    const auto throttle_is_down = _rx_chanvals[0] < THROTTLE_DOWN_MAX;

    static bool _armed;

    // Check failsafe via RX timeout
    if (_last_rx_usec > 0 &&
            usec_curr > _last_rx_usec &&
            usec_curr - _last_rx_usec > RX_TIMEOUT_USEC) {
        _armed = false;
    }

    // Push-button arming
    static float _chan5_prev;
    const auto chan5_curr = _rx_chanvals[4];
    if (_chan5_prev != 0 && _chan5_prev != chan5_curr) {
        _armed = _armed ? false : throttle_is_down ? true : _armed;
    }
    _chan5_prev = chan5_curr;

    hf::axis3_i16_t gyroRaw = {};
    hf::axis3_i16_t accelRaw = {};
    imu_device_read(
            gyroRaw.x, gyroRaw.y, gyroRaw.z,
            accelRaw.x, accelRaw.y, accelRaw.z);

    hf::Vec3 gyroDps = {}; // XXX should use returned value
    hf::Vec3 accelGs = {}; // XXX should use returned value
    const bool imuIsCalibrated =
        _imu.step( usec_curr/1000, gyroRaw, accelRaw, GYRO_SCALE, ACCEL_SCALE,
                gyroDps, accelGs);
    (void)imuIsCalibrated; // XXX should rapid-blink LED until IMU calibrated

    _ekf.enqueueImu(&gyroDps, &accelGs);

    const bool isFlying = true; // XXX

    const auto state = getVehicleState(isFlying, gyroDps);

    hf::Setpoint setpoint = {
        (_rx_chanvals[0]+1)/2,
        _rx_chanvals[1] * hf::PositionController::MAX_DEMAND_DEG, 
        _rx_chanvals[2] * hf::PositionController::MAX_DEMAND_DEG, 
        _rx_chanvals[3]};

#ifdef PROFILE
    profile();
#endif

#ifdef DEBUG
    debug(_armed, setpoint, state);
#endif

    static hf::StabilizerPid _stabilizerPid;
    _stabilizerPid = hf::StabilizerPid::run(_stabilizerPid, !throttle_is_down,
            dt, state, setpoint);

    static hf::Mixer _mixer;
    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(_armed, _mixer.motorvals);
}
