/*
   Hackflight for Teensy 4.0 

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
#include <dsmrx.hpp>  

// Hackflight library
#include <hackflight.h>
#include <firmware/estimators/newekf.hpp>
#include <firmware/newimu.hpp>
#include <datatypes.h>
#include <mixers/bfquadx.hpp>
#include <pidcontrol/newpids/position.hpp>
#include <pidcontrol/stabilizer.hpp>

#define DEBUG

// IMU ------------------------------------------------------------

static hf::IMU _imu;

// EKF ------------------------------------------------------------

static const uint32_t FREQ_EKF_PREDICTION = 100;

static hf::EKF _ekf;

// Receiver -------------------------------------------------------

static Dsm2048 _dsm2048;

// DSMX receiver callback
void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

// Motors ---------------------------------------------------------

// static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});
static DshotTeensy4 _motors = DshotTeensy4({2, 3, 4, 5});

// LED -------------------------------------------------------------

// static const uint8_t LED_PIN = 14; // external 
static const uint8_t LED_PIN = 13; // built-in

// Safety ----------------------------------------------------------

static const float ARMING_SWITCH_MIN = 0;
static const float THROTTLE_DOWN_MAX = -0.95;

// FAFO -----------------------------------------------------------

static const uint32_t LOOP_FREQ_HZ = 2000;

// Helper functions ------------------------------------------------

static void runDelayLoop(const uint32_t usec_curr)
{
    float invFreq = 1.0 / LOOP_FREQ_HZ * 1000000.0;
    uint32_t checker = micros();

    while (invFreq > (checker - usec_curr)) {
        checker = micros();
    }
}

static void blinkInLoop(const uint32_t usec_curr)
{
    static uint32_t blink_counter, blink_delay;
    static bool blinkAlternate;

    if (usec_curr - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(LED_PIN, blinkAlternate); 

        if (blinkAlternate == 1) {
            blinkAlternate = 0;
            blink_delay = 100000;
        }
        else if (blinkAlternate == 0) {
            blinkAlternate = 1;
            blink_delay = 2000000;
        }
    }
}

static void blinkOnStartup()
{
    for (int j = 1; j<= 3; j++) {
        digitalWrite(LED_PIN, LOW);
        delay(70);
        digitalWrite(LED_PIN, HIGH);
        delay(360);
    }
}

static void getVehicleState(const bool isFlying, hf::vehicleState_t & state)
{
    static hf::Timer _timer;

    static bool _didResetEstimation;

    const uint32_t nowMs = millis();

    if (_didResetEstimation) {
        _ekf.init(nowMs);
        _didResetEstimation = false;
    }

    // Run the system dynamics to predict the state forward.
    if (_timer.ready(FREQ_EKF_PREDICTION)) {
        _ekf.predict(nowMs, isFlying); 
    }

    // Get state estimate from EKF
    _ekf.getStateEstimate(nowMs, state);

    // Get angular velocities directly from gyro
    hf::axis3_t gyroData = {}; // XXX should use Vec3, returned value
    _imu.getGyroData(gyroData);
    state.dphi   = gyroData.x;
    state.dtheta = gyroData.y;
    state.dpsi   = -gyroData.z; // negate for nose-right positive
}

static bool readReceiver(
        const uint32_t usec_curr, float * channel_values)
{
    if (_dsm2048.timedOut(usec_curr)) {
        return false;
    }

    if (_dsm2048.gotNewFrame()) {
        _dsm2048.getChannelValues(channel_values, 6);
    }

    return true;
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

    pinMode(LED_PIN, OUTPUT); 

    digitalWrite(LED_PIN, HIGH);

    delay(5);

    Serial1.begin(115000);

    _imu.init();

    delay(10);

    _motors.arm(); 

    blinkOnStartup(); 
}

#ifdef DEBUG
static void debug(
        const bool imuIsCalibrated,
        const hf::vehicleState_t & state,
        const hf::Setpoint & setpoint)
{
    static uint32_t _msec;
    const auto msec = millis();

    if (msec - _msec > 20) {
        //printf("imu is calibrated: %d\n", imuIsCalibrated);
        printf("dpsi=%+3.3f\n", state.dpsi);
        _msec = msec;
    }
}
#endif

void loop()
{
    const auto usec_curr = micros();      

    const auto dt = getDt(usec_curr);

    blinkInLoop(usec_curr); 

    static float _channel_values[6];
    const auto failsafe = !readReceiver(usec_curr, _channel_values);

    const auto throttle_is_down = _channel_values[0] < THROTTLE_DOWN_MAX;

    static bool _armed;

    _armed = 
        failsafe ? false :
        _channel_values[4] < ARMING_SWITCH_MIN  ? false :
        throttle_is_down ? true :
        _armed;

    const bool imuIsCalibrated = _imu.step(&_ekf, usec_curr/1000);

    const bool isFlying = true; // XXX

    // XXX should be return value
    hf::vehicleState_t state = {};
    getVehicleState(isFlying, state);

    hf::Setpoint setpoint = {
        (_channel_values[0]+1)/2,
        _channel_values[1] * hf::PositionController::MAX_DEMAND_DEG, 
        _channel_values[2] * hf::PositionController::MAX_DEMAND_DEG, 
        _channel_values[3]};

#ifdef DEBUG
    debug(imuIsCalibrated, state, setpoint);
#endif

    static hf::StabilizerPid _stabilizerPid;
    _stabilizerPid = hf::StabilizerPid::run(_stabilizerPid, !throttle_is_down,
            dt, state, setpoint);

    static hf::Mixer _mixer;
    _mixer = hf::Mixer::run(_mixer, _stabilizerPid.setpoint);

    _motors.run(_armed, _mixer.motorvals);

    runDelayLoop(usec_curr); 
}
