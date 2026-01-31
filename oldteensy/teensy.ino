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

// Third-party libraries
#include <MPU6050.h>
#include <oneshot125.hpp>
#include <dsmrx.hpp>

// Hackflight library
#include <hackflight.h>
#include <datatypes.h>
#include <firmware/estimators/madgwick.hpp>
#include <mixers/bfquadx.hpp>
#include <pidcontrol.hpp>

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

static constexpr float ACCEL_ERROR_X  = 0;
static constexpr float ACCEL_ERROR_Y  = 0;
static constexpr float ACCEL_ERROR_Z  = 0;
static constexpr float GYRO_ERROR_X = 0;
static constexpr float GYRO_ERROR_Y = 0;
static constexpr float GYRO_ERROR_Z = 0;

// Misc. -----------------------------------------------------------

static constexpr float YAW_DEMAND_INC = 3e-6;

// Sensors
static MPU6050 _mpu6050;

// Motors
static OneShot125 _motors = OneShot125(MOTOR_PINS);

// Receiver
static float _channels[6];

// State estimation
static hf::MadgwickFilter  _madgwick;

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

static void runMotors(const float * motors, const bool mode)
{
    // Rescale motor values for OneShot125
    auto m1_usec = scaleMotor(motors[0]);
    auto m2_usec = scaleMotor(motors[1]);
    auto m3_usec = scaleMotor(motors[2]);
    auto m4_usec = scaleMotor(motors[3]);

    // Shut motors down when not armed
    if (mode != hf::MODE_ARMED) {

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
}

float getDt()
{
    const auto usec_curr = micros();      
    static uint32_t _usec_prev;
    const auto dt = (usec_curr - _usec_prev)/1000000.0;
    _usec_prev = usec_curr;      
    return dt;
}


static void getDemands(hf::demands_t & demands, hf::mode_e & mode)
{
    // Read channels values from receiver
    if (_dsm2048.timedOut(micros())) {

        mode = hf::MODE_PANIC;
    }
    else if (_dsm2048.gotNewFrame()) {

        _dsm2048.getChannelValuesMlp6Dsm(_channels);
    }

    // When throttle is down, toggle arming on switch press/release
    const auto is_arming_switch_on = _channels[4] > ARMING_SWITCH_MIN;

    if (_channels[0] < THROTTLE_ARMING_MAX && is_arming_switch_on) {
        mode = mode == hf::MODE_IDLE ? hf::MODE_ARMED : mode;
    }

    if (!is_arming_switch_on) {
        mode = mode == hf::MODE_ARMED ? hf::MODE_IDLE : mode;
    }

    // Convert stick demands to appropriate intervals
    demands.thrust = (_channels[0] + 1) / 2; // -1,+1 => 0,1
    demands.roll  = _channels[1] * PITCH_ROLL_PRESCALE;
    demands.pitch = _channels[2] * PITCH_ROLL_PRESCALE;
    demands.yaw   = _channels[3] * YAW_PRESCALE;
}

static void getVehicleState(const float dt, hf::vehicleState_t & state)
{
    // Read IMU
    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Accelerometer Gs
    const hf::axis3_t accel = {
        -ax / ACCEL_SCALE_FACTOR - ACCEL_ERROR_X,
        ay / ACCEL_SCALE_FACTOR - ACCEL_ERROR_Y,
        az / ACCEL_SCALE_FACTOR - ACCEL_ERROR_Z
    };

    // Gyro deg /sec
    const hf::axis3_t gyro = {
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
}

static void debug(
        const hf::vehicleState_t & state,
        const hf::demands_t & demands,
        const float * motors)
{
    static uint32_t _msec_prev;
    const auto msec_curr  = millis();
    if (msec_curr - _msec_prev > 10) {
        printf( 
                "phi=%+3.3f theta=%+3.3f psi=%+3.3f => "
                "r=%+3.3f p=%+3.3f y=%+3.3f => "
                "m1+%3.3f m2=%3.3f m3=%3.3f m4=%3.3f\n",
                state.phi, state.theta, state.psi,
                demands.roll, demands.pitch, demands.yaw,
                motors[0], motors[1], motors[2], motors[3]);
        _msec_prev = msec_curr;
    }
}


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

    // Initialize state estimator
    _madgwick.initialize();

    // Arm OneShot125 motors
    _motors.arm();

    // Initialized LED
    pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
    static hf::mode_e _mode;

    const float dt = getDt();

    hf::demands_t demands = {};
    getDemands(demands, _mode);

    digitalWrite(LED_PIN, _mode == hf::MODE_ARMED ? HIGH : LOW);

    hf::vehicleState_t state = {};
    getVehicleState(dt, state);

    hf::PidControl::runStabilizerPids(dt, YAW_DEMAND_INC, state, demands, demands);

    // Support same rate controllers as Crazyflie
    demands.roll /= 500000;
    demands.pitch /= 500000;
    demands.yaw /= 500000;

    float motors[4] = {};

    hf::Mixer::mix(demands, motors);

    (void)debug/*(state, demands, motors)*/;

    runMotors(motors, _mode);

    runLoopDelay(micros());
}
