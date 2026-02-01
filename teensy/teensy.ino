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
#include <teensy_pidcontrol.hpp>

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


// Open-loop demands ----------------------------------------------

static constexpr float MAX_PITCH_ROLL_DEMAND_DEG = 30;     
static constexpr float MAX_YAW_DEMAND_DPS = 160;     

// IMU ------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384;

// LED -------------------------------------------------------------

static const uint8_t LED_PIN = 14;

// Safety ----------------------------------------------------------

static const uint16_t ARMING_SWITCH_MIN = 1500;
static const uint16_t THROTTLE_DOWN_MAX = 1050;

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

static void initImu() {

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

static void getOpenLoopDemands(const uint16_t * channel_pwms,
        hf::demands_t & demands)
{
    demands.thrust = (channel_pwms[0] - 1000.0)/1000.0; 
    demands.thrust = constrain(demands.thrust, 0.0, 1.0); 

    demands.roll = (channel_pwms[1] - 1500.0)/500.0; 
    demands.roll = constrain(demands.roll, -1.0, 1.0)*MAX_PITCH_ROLL_DEMAND_DEG; 

    demands.pitch = (channel_pwms[2] - 1500.0)/500.0; 
    demands.pitch = constrain(demands.pitch, -1.0, 1.0)*MAX_PITCH_ROLL_DEMAND_DEG; 

    demands.yaw = (channel_pwms[3] - 1500.0)/500.0; 
    demands.yaw = constrain(demands.yaw, -1.0, 1.0)*MAX_YAW_DEMAND_DPS; 
}

static void runDelayLoop(const unsigned long current_time)
{

    float invFreq = 1.0 / LOOP_FREQ_HZ * 1000000.0;
    unsigned long checker = micros();

    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}

static void loopBlink(const unsigned long current_time) {

    static unsigned long blink_counter, blink_delay;
    static bool blinkAlternate;

    if (current_time - blink_counter > blink_delay) {
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

static void setupBlink(int numBlinks,int upTime, int downTime) {

    for (int j = 1; j<= numBlinks; j++) {
        digitalWrite(LED_PIN, LOW);
        delay(downTime);
        digitalWrite(LED_PIN, HIGH);
        delay(upTime);
    }
}

// Main ----------------------------------------------------------------------

void setup()
{
    Serial.begin(500000); 

    delay(500);

    pinMode(LED_PIN, OUTPUT); 

    digitalWrite(LED_PIN, HIGH);

    delay(5);

    Serial1.begin(115000);

    initImu();

    _madgwick.initialize();

    delay(10);

    _motors.arm(); 

    setupBlink(3,160,70); 
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
        const uint32_t current_time, uint16_t * channel_pwms)
{
    if (_dsm2048.timedOut(current_time)) {
        return false;
    }

    if (_dsm2048.gotNewFrame()) {
        _dsm2048.getChannelValues(channel_pwms, 6);
    }

    return true;
}

static float getDt(const uint32_t current_time)
{
    static unsigned long _prev_time;
    const float dt = (current_time - _prev_time)/1000000.0;
    _prev_time = current_time;

    return dt;
}

void loop()
{
    const unsigned long current_time = micros();      

    const float dt = getDt(current_time);

    loopBlink(current_time); 
    
    static uint16_t _channel_pwms[6];
    const bool failsafe = !readReceiver(current_time, _channel_pwms);

    const bool throttle_is_down = _channel_pwms[0] < THROTTLE_DOWN_MAX;

    static bool _armed;

    _armed = 
        failsafe ? false :
        _channel_pwms[4] < ARMING_SWITCH_MIN  ? false :
        throttle_is_down ? true :
        _armed;

    hf::demands_t open_loop_demands = {};
    getOpenLoopDemands(_channel_pwms, open_loop_demands);

    hf::vehicleState_t state = {};
    getVehicleState(dt, state);

    hf::demands_t pid_demands = {open_loop_demands.thrust, 0, 0, 0};
    hf::PidControl::run(dt, throttle_is_down, state, open_loop_demands,
            pid_demands);

    float motorvals[4] = {};
    hf::Mixer::mix(pid_demands, motorvals);

    _motors.run(_armed, motorvals);

    runDelayLoop(current_time); 
}

