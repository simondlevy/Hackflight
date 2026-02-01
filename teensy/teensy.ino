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

// Motors
static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});


// IMU ------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384;

// LED -------------------------------------------------------------
static const uint8_t LED_PIN = 14;

// Failsafe --------------------------------------------------------

static const uint16_t CHANNEL_1_FAILSAFE = 1000; 
static const uint16_t CHANNEL_2_FAILSAFE = 1500; 
static const uint16_t CHANNEL_3_FAILSAFE = 1500; 
static const uint16_t CHANNEL_4_FAILSAFE = 1500; 
static const uint16_t CHANNEL_5_FAILSAFE = 2000; 
static const uint16_t CHANNEL_6_FAILSAFE = 2000; 

// IMU -------------------------------------------------------------

static constexpr float B_ACCEL = 0.14;     
static constexpr float B_GYRO = 0.1;       

static constexpr float ACCEL_ERROR_X = 0.0;
static constexpr float ACCEL_ERROR_Y = 0.0;
static constexpr float ACCEL_ERROR_Z = 0.0;
static constexpr float GYRO_ERROR_X = 0.0;
static constexpr float GYRO_ERROR_Y= 0.0;
static constexpr float GYRO_ERROR_Z = 0.0;

// PIDs ------------------------------------------------------------

static unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm,
                     channel_4_pwm, channel_5_pwm, channel_6_pwm;

static float AccX, AccY, AccZ;
static float GyroX, GyroY, GyroZ;

static float thro_des, roll_des, pitch_des, yaw_des;
static float roll_passthru, pitch_passthru, yaw_passthru;

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

static void readImu() {

    static float AccX_prev, AccY_prev, AccZ_prev;
    static float GyroX_prev, GyroY_prev, GyroZ_prev;

    int16_t AcX=0, AcY=0, AcZ=0, GyX=0, GyY=0, GyZ=0;

    _mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    AccX = AcX / ACCEL_SCALE_FACTOR; 

    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;

    AccX = AccX - ACCEL_ERROR_X;
    AccY = AccY - ACCEL_ERROR_Y;
    AccZ = AccZ - ACCEL_ERROR_Z;

    AccX = (1.0 - B_ACCEL)*AccX_prev + B_ACCEL*AccX;
    AccY = (1.0 - B_ACCEL)*AccY_prev + B_ACCEL*AccY;
    AccZ = (1.0 - B_ACCEL)*AccZ_prev + B_ACCEL*AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    GyroX = GyX / GYRO_SCALE_FACTOR; 

    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    GyroX = GyroX - GYRO_ERROR_X;
    GyroY = GyroY - GYRO_ERROR_Y;
    GyroZ = GyroZ - GYRO_ERROR_Z;

    GyroX = (1.0 - B_GYRO)*GyroX_prev + B_GYRO*GyroX;
    GyroY = (1.0 - B_GYRO)*GyroY_prev + B_GYRO*GyroY;
    GyroZ = (1.0 - B_GYRO)*GyroZ_prev + B_GYRO*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
}

static void getDesState()
{

    static constexpr float maxRoll = 30.0;     
    static constexpr float maxPitch = 30.0;    
    static constexpr float maxYaw = 160.0;     

    thro_des = (channel_1_pwm - 1000.0)/1000.0; 
    roll_des = (channel_2_pwm - 1500.0)/500.0; 
    pitch_des = (channel_3_pwm - 1500.0)/500.0; 
    yaw_des = (channel_4_pwm - 1500.0)/500.0; 
    pitch_passthru = pitch_des/2.0; 
    yaw_passthru = yaw_des/2.0; 
    thro_des = constrain(thro_des, 0.0, 1.0); 
    roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; 
    pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; 
    yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; 
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

static void runPids(
        const float dt, const float phi, const float theta, const float psi,
        float & roll_PID, float & pitch_PID, float & yaw_PID)
{

    static constexpr float i_limit = 25.0;     
    static constexpr float Kp_roll_angle = 0.2;    
    static constexpr float Ki_roll_angle = 0.3;    
    static constexpr float Kd_roll_angle = 0.05;   
    static constexpr float Kp_pitch_angle = 0.2;   
    static constexpr float Ki_pitch_angle = 0.3;   
    static constexpr float Kd_pitch_angle = 0.05;  
    static constexpr float Kp_yaw = 0.3;           
    static constexpr float Ki_yaw = 0.05;          
    static constexpr float Kd_yaw = 0.00015;       

    static float integral_roll, integral_roll_prev, derivative_roll;
    static float integral_pitch, integral_pitch_prev, derivative_pitch;
    static float error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw;

    const float error_roll = roll_des - phi;
    integral_roll = integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   

        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -i_limit, i_limit); 

    derivative_roll = GyroX;
    roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll
            - Kd_roll_angle*derivative_roll); 

    const float error_pitch = pitch_des - theta;
    integral_pitch = integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   

        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit); 

    derivative_pitch = GyroY;
    pitch_PID = .01*(
            Kp_pitch_angle*error_pitch +
            Ki_pitch_angle*integral_pitch -
            Kd_pitch_angle*derivative_pitch); 

    // Negate gyro Z for nose-right positive
    const auto dpsi = -GyroZ;

    const float error_yaw = yaw_des - dpsi;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   

        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit); 

    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); 

    integral_roll_prev = integral_roll;

    integral_pitch_prev = integral_pitch;

    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
}

static void getCommands() {

    if (_dsm2048.timedOut(micros())) {

    }
    else if (_dsm2048.gotNewFrame()) {
        uint16_t values[6];
        _dsm2048.getChannelValues(values, 6);

        channel_1_pwm = values[0];
        channel_2_pwm = values[1];
        channel_3_pwm = values[2];
        channel_4_pwm = values[3];
        channel_5_pwm = values[4];
        channel_6_pwm = values[5];
    }
}

static void failSafe() {

    static const uint16_t MINVAL = 800;
    static const uint16_t MAXVAL = 2200;

    int check1 = 0;
    int check2 = 0;
    int check3 = 0;
    int check4 = 0;
    int check5 = 0;
    int check6 = 0;

    if (channel_1_pwm > MAXVAL || channel_1_pwm < MINVAL) check1 = 1;
    if (channel_2_pwm > MAXVAL || channel_2_pwm < MINVAL) check2 = 1;
    if (channel_3_pwm > MAXVAL || channel_3_pwm < MINVAL) check3 = 1;
    if (channel_4_pwm > MAXVAL || channel_4_pwm < MINVAL) check4 = 1;
    if (channel_5_pwm > MAXVAL || channel_5_pwm < MINVAL) check5 = 1;
    if (channel_6_pwm > MAXVAL || channel_6_pwm < MINVAL) check6 = 1;

    if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
        channel_1_pwm = CHANNEL_1_FAILSAFE;
        channel_2_pwm = CHANNEL_2_FAILSAFE;
        channel_3_pwm = CHANNEL_3_FAILSAFE;
        channel_4_pwm = CHANNEL_4_FAILSAFE;
        channel_5_pwm = CHANNEL_5_FAILSAFE;
        channel_6_pwm = CHANNEL_6_FAILSAFE;
    }
}

static void loopRate(int freq, const unsigned long current_time)
{

    float invFreq = 1.0/freq*1000000.0;
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

void setup()
{
    Serial.begin(500000); 

    delay(500);

    pinMode(LED_PIN, OUTPUT); 

    digitalWrite(LED_PIN, HIGH);

    delay(5);

    Serial1.begin(115000);

    channel_1_pwm = CHANNEL_1_FAILSAFE;
    channel_2_pwm = CHANNEL_2_FAILSAFE;
    channel_3_pwm = CHANNEL_3_FAILSAFE;
    channel_4_pwm = CHANNEL_4_FAILSAFE;
    channel_5_pwm = CHANNEL_5_FAILSAFE;
    channel_6_pwm = CHANNEL_6_FAILSAFE;

    initImu();

    _madgwick.initialize();

    delay(10);

    _motors.arm(); 

    setupBlink(3,160,70); 
}

void loop()
{
    static unsigned long _prev_time;
    const unsigned long current_time = micros();      
    const float dt = (current_time - _prev_time)/1000000.0;
    _prev_time = current_time;

    loopBlink(current_time); 

    static bool _armed;

    if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050)) {
        _armed = true;
    }

    readImu(); 

    float phi=0, theta=0, psi=0;

    const hf::axis3_t gyro = {GyroX, -GyroY, -GyroZ}; 
    const hf::axis3_t accel = {-AccX, AccY, AccZ}; 
    _madgwick.getEulerAngles(dt, gyro, accel, phi, theta, psi);
    psi = -psi; // make nose-right positive

    getDesState(); 

    float roll_PID=0, pitch_PID=0, yaw_PID=0;

    runPids(dt, phi, theta, psi, roll_PID, pitch_PID, yaw_PID); 

    const hf::demands_t demands = {thro_des, roll_PID, pitch_PID, yaw_PID};
    float motorvals[4] = {};
    hf::Mixer::mix(demands, motorvals);

    if ((channel_5_pwm < 1500) || (_armed == false)) {
        _armed = false;
    }

    _motors.run(_armed, motorvals);

    getCommands(); 

    failSafe(); 

    loopRate(2000, current_time); 
}

