/*
   Hackflight for Teensy 4.0 

   Based on  https://github.com/nickrehm/dRehmFlight

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
#include <SPI.h>

// Third-party libraries
#include <MPU6050.h>
#include <oneshot125.hpp>
#include <dsmrx.hpp>

// Hackflight library
#include <hackflight.h>
#include <datatypes.h>
#include <firmware/estimators/madgwick.hpp>
#include <mixers/bfquadx.hpp>
#include <pids/pitchroll_angle.hpp>
#include <pids/pitchroll_rate.hpp>

#include "yaw_rate_pid.hpp"

static Dsm2048 _dsm2048;

// DSMX receiver callback
void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

// FAFO -----------------------------------------------------------
static const uint32_t LOOP_FREQ_HZ = 2000;

// IMU ------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131.0;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384.0;

// LED
static const uint8_t LED_PIN = 14;
static constexpr float FAILSAFE_BLINK_RATE_HZ = 3;

// Margins of safety for arming
static constexpr float THROTTLE_ARMING_MAX = -0.9;
static constexpr float ARMING_SWITCH_MIN = 0.9;

// Motors ---------------------------------------------------------
const std::vector<uint8_t> MOTOR_PINS = { 6, 5, 4, 3 };

// Max pitch angle in degrees for angle mode (maximum ~70 degrees),
// deg/sec for rate mode
static constexpr float PITCH_ROLL_PRESCALE = 30.0;    

// Max yaw rate in deg/sec
static constexpr float YAW_PRESCALE = 160.0;     

// IMU calibration parameters -------------------------------------

static constexpr float ACC_ERROR_X  = -0.030;
static constexpr float ACC_ERROR_Y  = +0.025;
static constexpr float ACC_ERROR_Z  = -0.11;
static constexpr float GYRO_ERROR_X = -3.3;
static constexpr float GYRO_ERROR_Y = -0.50;
static constexpr float GYRO_ERROR_Z = +0.6875;

// Sensors
static MPU6050 _mpu6050;

// Motors
static OneShot125 _motors = OneShot125(MOTOR_PINS);

// Timing
static uint32_t _usec_curr;

// Receiver
static float _channels[6];

// State estimation
static MadgwickFilter  _madgwick;

// Safety
static mode_e _mode;

static void runLoopDelay(const uint32_t usec_curr)
{
    //DESCRIPTION: Regulate main loop rate to specified frequency
    //in Hz
    /*
     * It's good to operate at a constant loop rate for filters to
     * remain stable and whatnot. Interrupt routines running in the
     * background cause the loop rate to fluctuate. This function
     * basically just waits at the end of every loop iteration
     * until the correct time has passed since the start of the
     * current loop for the desired loop rate in Hz. 2kHz is a good
     * rate to be at because the loop nominally will run between
     * 2.8kHz - 4.2kHz. This lets us have a little room to add
     * extra computations and remain above 2kHz, without needing to
     * retune all of our filtering parameters.
     */
    float invFreq = 1.0/LOOP_FREQ_HZ*1000000.0;

    unsigned long checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - usec_curr)) {
        checker = micros();
    }
}

static uint8_t u8constrain(
        const uint8_t val, const uint8_t minval, const uint8_t maxval)
{
    return val < minval ? minval : val > maxval ? maxval : val;
}

static uint8_t scaleMotor(const float mval)
{
    return u8constrain(mval*125 + 125, 125, 250);

}

static void reportForever(const char * message)
{
    while (true) {
        printf("%s\n", message);
        delay(500);
    }

}

static void initImu() 
{
    //Note this is 2.5 times the spec sheet 400 kHz max...
    Wire.setClock(1000000); 

    _mpu6050.initialize();

    if (!_mpu6050.testConnection()) {
        reportForever("MPU6050 initialization unsuccessful\n");
    }

    // From the reset state all registers should be 0x00, so we
    // should be at max sample rate with digital low pass filter(s)
    // off.  All we need to do is set the desired fullscale ranges
    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

static void runMotors(const float * motors, const bool safeMode=true)
{
    // Rescale motor values for OneShot125
    auto m1_usec = scaleMotor(motors[0]);
    auto m2_usec = scaleMotor(motors[1]);
    auto m3_usec = scaleMotor(motors[2]);
    auto m4_usec = scaleMotor(motors[3]);

    // Shut motors down when not armed
    if (safeMode && _mode != MODE_ARMED) {

        m1_usec = 120;
        m2_usec = 120;
        m3_usec = 120;
        m4_usec = 120;
    }

    // Run motors
    _motors.set(0, m1_usec);
    _motors.set(1, m2_usec);
    _motors.set(2, m3_usec);
    _motors.set(3, m4_usec);

    _motors.run();

    runLoopDelay(_usec_curr);
}


void readData(float & dt, demands_t & demands, vehicleState_t & state)
{
    // Keep track of what time it is and how much time has elapsed
    // since the last loop
    _usec_curr = micros();      
    static uint32_t _usec_prev;
    dt = (_usec_curr - _usec_prev)/1000000.0;
    _usec_prev = _usec_curr;      

    // Read channels values from receiver
    if (_dsm2048.timedOut(micros())) {

        _mode = MODE_PANIC;
    }
    else if (_dsm2048.gotNewFrame()) {

        _dsm2048.getChannelValuesMlp6Dsm(_channels);
    }

    // When throttle is down, toggle arming on switch press/release
    const auto is_arming_switch_on = _channels[4] > ARMING_SWITCH_MIN;

    if (_channels[0] < THROTTLE_ARMING_MAX && is_arming_switch_on) {
        _mode = _mode == MODE_IDLE ? MODE_ARMED : _mode;
    }

    if (!is_arming_switch_on) {
        _mode = _mode == MODE_ARMED ? MODE_IDLE : _mode;
    }

    // Read IMU
    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Accelerometer Gs
    const axis3_t accel = {
        -ax / ACCEL_SCALE_FACTOR - ACC_ERROR_X,
        ay / ACCEL_SCALE_FACTOR - ACC_ERROR_Y,
        az / ACCEL_SCALE_FACTOR - ACC_ERROR_Z
    };

    // Gyro deg /sec
    const axis3_t gyro = {
        gx / GYRO_SCALE_FACTOR - GYRO_ERROR_X, 
        -gy / GYRO_SCALE_FACTOR - GYRO_ERROR_Y,
        -gz / GYRO_SCALE_FACTOR - GYRO_ERROR_Z
    };

    // Run state estimator to get Euler angles
    _madgwick.getEulerAngles(dt, gyro, accel,
         state.phi, state.theta, state.psi);

    // Get angular velocities directly from gyro
    state.dphi = gyro.x;
    state.dtheta = -gyro.y;
    state.dpsi = gyro.z;

    // Convert stick demands to appropriate intervals
    demands.thrust = (_channels[0] + 1) / 2; // -1,+1 => 0,1
    demands.roll  = _channels[1] * PITCH_ROLL_PRESCALE;
    demands.pitch = _channels[2] * PITCH_ROLL_PRESCALE;
    demands.yaw   = _channels[3] * YAW_PRESCALE;

    // Use LED to indicate arming mode
    digitalWrite(LED_PIN, _mode == MODE_ARMED ? HIGH : LOW);
}

static constexpr float THROTTLE_DOWN = 0.06;

//////////////////////////////////////////////////////////////////////////////

void setup() 
{
    // Initialize the I^2C bus
    Wire.begin();

    // Initialize the I^2C sensors
    initImu();

    // Set up serial debugging
    Serial.begin(115200);

    // Set up serial connection from DSMX receiver
    Serial1.begin(115200);

    // Set up serial connection with Raspberry Pi
    Serial4.begin(115200);

    // Initialize state estimator
    _madgwick.initialize();

    // Arm OneShot125 motors
    _motors.arm();

    pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
    float dt=0;
    demands_t demands = {};
    vehicleState_t state = {};

    readData(dt, demands, state);

    const auto airborne = demands.thrust > 0;

    PitchRollAngleController::run(
            airborne,
            dt,
            state.phi, state.theta,
            demands.roll, demands.pitch,
            demands.roll, demands.pitch);

    PitchRollRateController::run(
            airborne,
            dt,
            state.dphi, state.dtheta,
            demands.roll, demands.pitch,
            demands.roll, demands.pitch);

    demands.yaw = YawRateController::run(
        airborne, dt, state.dpsi, demands.yaw);

    // Support same rate controllers as Crazyflie
    demands.roll /= 500000;
    demands.pitch /= 500000;

    float motors[4] = {};

    Mixer::mix(demands, motors);

    runMotors(motors);
}
