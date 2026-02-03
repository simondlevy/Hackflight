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
#include <dshot-teensy4.hpp>  
#include <dsmrx.hpp>  
#include <MPU6050.h>

// Hackflight library
#include <hackflight.h>
#include <datatypes.h>
#include <firmware/estimators/madgwick.hpp>
#include <mixers/bfquadx.hpp>
#include <new_pidcontrol.hpp>

static MPU6050 _mpu6050;

static Dsm2048 _dsm2048;

// State estimation
static hf::MadgwickFilter  _madgwick;

// DSMX receiver callback
void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

// Motors ---------------------------------------------------------

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

// IMU ------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384;

// LED -------------------------------------------------------------

static const uint8_t LED_PIN = 14;

// Safety ----------------------------------------------------------

static const float ARMING_SWITCH_MIN = 0;
static const float THROTTLE_DOWN_MAX = -0.95;

// IMU -------------------------------------------------------------

static constexpr float B_ACCEL = 0.14;     
static constexpr float B_GYRO = 0.1;       

static constexpr float ACCEL_ERROR_X = 0.0;
static constexpr float ACCEL_ERROR_Y = 0.0;
static constexpr float ACCEL_ERROR_Z = 0.0;
static constexpr float GYRO_ERROR_X = 0.0;
static constexpr float GYRO_ERROR_Y= 0.0;
static constexpr float GYRO_ERROR_Z = 0.0;

// FAFO -----------------------------------------------------------

static const uint32_t LOOP_FREQ_HZ = 2000;

// Helper functions ------------------------------------------------

static void initImu()
{

    Wire.begin();
    Wire.setClock(1000000); 

    _mpu6050.initialize();

    if (_mpu6050.testConnection() == false) {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(1) {}
    }

    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

static void readImu(
        float & accel_x, float & accel_y, float & accel_z,
        float & gyro_x, float & gyro_y, float & gyro_z)
{
    static float accel_x_prev, accel_y_prev, accel_z_prev;
    static float gyro_x_prev, gyro_y_prev, gyro_z_prev;

    int16_t AcX=0, AcY=0, AcZ=0, GyX=0, GyY=0, GyZ=0;

    _mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    accel_x = AcX / ACCEL_SCALE_FACTOR; 
    accel_y = AcY / ACCEL_SCALE_FACTOR;
    accel_z = AcZ / ACCEL_SCALE_FACTOR;

    accel_x = accel_x - ACCEL_ERROR_X;
    accel_y = accel_y - ACCEL_ERROR_Y;
    accel_z = accel_z - ACCEL_ERROR_Z;

    accel_x = (1.0 - B_ACCEL)*accel_x_prev + B_ACCEL*accel_x;
    accel_y = (1.0 - B_ACCEL)*accel_y_prev + B_ACCEL*accel_y;
    accel_z = (1.0 - B_ACCEL)*accel_z_prev + B_ACCEL*accel_z;
    accel_x_prev = accel_x;
    accel_y_prev = accel_y;
    accel_z_prev = accel_z;

    gyro_x = GyX / GYRO_SCALE_FACTOR; 
    gyro_y = GyY / GYRO_SCALE_FACTOR;
    gyro_z = GyZ / GYRO_SCALE_FACTOR;

    gyro_x = gyro_x - GYRO_ERROR_X;
    gyro_y = gyro_y - GYRO_ERROR_Y;
    gyro_z = gyro_z - GYRO_ERROR_Z;

    gyro_x = (1.0 - B_GYRO)*gyro_x_prev + B_GYRO*gyro_x;
    gyro_y = (1.0 - B_GYRO)*gyro_y_prev + B_GYRO*gyro_y;
    gyro_z = (1.0 - B_GYRO)*gyro_z_prev + B_GYRO*gyro_z;

    gyro_x_prev = gyro_x;
    gyro_y_prev = gyro_y;
    gyro_z_prev = gyro_z;
}

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

static void getVehicleState(const float dt, hf::vehicleState_t & state)
{
    float accel_x=0, accel_y=0, accel_z=0;
    float gyro_x=0, gyro_y=0, gyro_z=0;
    readImu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z); 

    _madgwick.getEulerAngles(dt,
            gyro_x, -gyro_y, -gyro_z,
            -accel_x, accel_y, accel_z,
            state.phi, state.theta, state.psi);

    state.dphi = gyro_x;
    state.dtheta = gyro_y;
    state.dpsi = -gyro_z;
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

    initImu();

    _madgwick.initialize();

    delay(10);

    _motors.arm(); 

    blinkOnStartup(); 
}

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

    hf::vehicleState_t state = {};
    getVehicleState(dt, state);

    hf::demands_t pid_demands = { (_channel_values[0]+1)/2, 0, 0, 0 };

    static hf::PidControl _pidControl;

    _pidControl.runStabilizer(
            dt,
            !throttle_is_down, 
            _channel_values[1] * hf::PositionController::MAX_DEMAND_DEG, 
            _channel_values[2] * hf::PositionController::MAX_DEMAND_DEG, 
            _channel_values[3],
            state,
            pid_demands);

    float motorvals[4] = {};
    hf::Mixer::mix(pid_demands, motorvals);

    _motors.run(_armed, motorvals);

    runDelayLoop(usec_curr); 
}
