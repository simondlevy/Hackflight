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

#include <Wire.h> 

#include <dsmrx.hpp>  
#include <MPU6050.h>

#define GYRO_250DPS //Default
                    //#define GYRO_500DPS
                    //#define GYRO_1000DPS
                    //#define GYRO_2000DPS

#define ACCEL_2G //Default
                 //#define ACCEL_4G
                 //#define ACCEL_8G
                 //#define ACCEL_16G


static MPU6050 _mpu6050;

static Dsm2048 _dsm2048;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

//Setup gyro and accel full scale value selection and scale factor

#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
static unsigned long channel_1_fs = 1000; //thro
static unsigned long channel_2_fs = 1500; //ail
static unsigned long channel_3_fs = 1500; //elev
static unsigned long channel_4_fs = 1500; //rudd
static unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
static unsigned long channel_6_fs = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
static float B_madgwick = 0.04;  //Madgwick filter parameter
static float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
static float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
static float AccErrorX = 0.0;
static float AccErrorY = 0.0;
static float AccErrorZ = 0.0;
static float GyroErrorX = 0.0;
static float GyroErrorY= 0.0;
static float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
static float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
static float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
static float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
static float maxYaw = 160.0;     //Max yaw rate in deg/sec

static float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
static float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
static float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
static float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
static float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
static float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)

static float Kp_yaw = 0.3;           //Yaw P-gain
static float Ki_yaw = 0.05;          //Yaw I-gain
static float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

static const int m1Pin = 6;
static const int m2Pin = 5;
static const int m3Pin = 4;
static const int m4Pin = 3;


//General stuff
static float dt;
static unsigned long current_time, prev_time;
static unsigned long blink_counter, blink_delay;
static bool blinkAlternate;

//Radio communication:
static unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
static unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;


//IMU:
static float AccX, AccY, AccZ;
static float AccX_prev, AccY_prev, AccZ_prev;
static float GyroX, GyroY, GyroZ;
static float GyroX_prev, GyroY_prev, GyroZ_prev;
static float roll_IMU, pitch_IMU, yaw_IMU;

static float q0 = 1.0f; //Initialize quaternion for madgwick filter
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

//Normalized desired state:
static float thro_des, roll_des, pitch_des, yaw_des;
static float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
static float error_roll, integral_roll, integral_roll_prev, derivative_roll,
             roll_PID;

static float error_pitch, integral_pitch, integral_pitch_prev,
             derivative_pitch, pitch_PID;

static float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev,
             derivative_yaw, yaw_PID;

//Mixer
static float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;

static int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

//Flight status
static bool armedFly;


static void controlMixer() {

    //Quad mixing - EXAMPLE
    m1_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
    m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
    m3_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
    m4_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
}

static void armedStatus() {

    //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
    if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050)) {
        armedFly = true;
    }
}

static void IMUinit() {
    //DESCRIPTION: Initialize IMU
    /*
     * Don't worry about how this works.
     */
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

    _mpu6050.initialize();

    if (_mpu6050.testConnection() == false) {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

static void getIMUdata() {
    //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
    /*
     * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
     * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
     * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
     * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
     * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
     * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
     */
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

    _mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    //Accelerometer
    AccX = AcX / ACCEL_SCALE_FACTOR; //G's
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccZ = AccZ - AccErrorZ;
    //LP filter accelerometer data
    AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
    AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
    AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    //Gyro
    GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    //LP filter gyro data
    GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
    GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
    GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;

}

static void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        //Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        //Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    //Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //Compute angles
    roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
    yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

static void getDesState() {
    //DESCRIPTION: Normalizes desired control values to appropriate values
    /*
     * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
     * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
     * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
     * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
     * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
     */
    thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
    roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
    pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
    yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
    pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
    yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5

    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
    pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

static void controlANGLE() {
    //DESCRIPTION: Computes control commands based on state error (angle)
    /*
     * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
     * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
     * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
     * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
     * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
     * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
     * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
     * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
     */

    //Roll
    error_roll = roll_des - roll_IMU;
    integral_roll = integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    derivative_roll = GyroX;
    roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    derivative_pitch = GyroY;
    pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw, stablize on rate from GyroZ
    error_yaw = -yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    integral_roll_prev = integral_roll;
    //Update pitch variables
    integral_pitch_prev = integral_pitch;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
}

static void scaleCommands() {
    //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
    /*
     * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
     * the mixer function are scaled to 0-180 for the servo library using standard PWM.
     * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
     * which are used to command the servos.
     */
    //Scaled to 125us - 250us for oneshot125 protocol
    m1_command_PWM = m1_command_scaled*125 + 125;
    m2_command_PWM = m2_command_scaled*125 + 125;
    m3_command_PWM = m3_command_scaled*125 + 125;
    m4_command_PWM = m4_command_scaled*125 + 125;

    //Constrain commands to motors within oneshot125 bounds
    m1_command_PWM = constrain(m1_command_PWM, 125, 250);
    m2_command_PWM = constrain(m2_command_PWM, 125, 250);
    m3_command_PWM = constrain(m3_command_PWM, 125, 250);
    m4_command_PWM = constrain(m4_command_PWM, 125, 250);
}

static void getCommands() {

    if (_dsm2048.timedOut(micros())) {
        //Serial.println("*** DSM RX TIMED OUT ***");
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

    //Low-pass the critical commands and update previous values
    float b = 0.7; //Lower=slower, higher=noiser
    channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
    channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
    channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
    channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
    channel_1_pwm_prev = channel_1_pwm;
    channel_2_pwm_prev = channel_2_pwm;
    channel_3_pwm_prev = channel_3_pwm;
    channel_4_pwm_prev = channel_4_pwm;
}

static void failSafe() {
    //DESCRIPTION: If radio gives garbage values, set all commands to default values
    /*
     * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
     * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
     * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
     * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
     * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
     */
    unsigned minVal = 800;
    unsigned maxVal = 2200;
    int check1 = 0;
    int check2 = 0;
    int check3 = 0;
    int check4 = 0;
    int check5 = 0;
    int check6 = 0;

    //Triggers for failure criteria
    if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
    if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
    if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
    if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
    if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
    if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

    //If any failures, set to default failsafe values
    if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
        channel_1_pwm = channel_1_fs;
        channel_2_pwm = channel_2_fs;
        channel_3_pwm = channel_3_fs;
        channel_4_pwm = channel_4_fs;
        channel_5_pwm = channel_5_fs;
        channel_6_pwm = channel_6_fs;
    }
}

static void commandMotors() {
    //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
    /*
     * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
     * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
     */
    int wentLow = 0;
    int pulseStart, timer;
    int flagM1 = 0;
    int flagM2 = 0;
    int flagM3 = 0;
    int flagM4 = 0;

    //Write all motor pins high
    digitalWrite(m1Pin, HIGH);
    digitalWrite(m2Pin, HIGH);
    digitalWrite(m3Pin, HIGH);
    digitalWrite(m4Pin, HIGH);
    pulseStart = micros();

    //Write each motor pin low as correct pulse length is reached
    while (wentLow < 4 ) { //Keep going until final (6th) pulse is finished, then done
        timer = micros();
        if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
            digitalWrite(m1Pin, LOW);
            wentLow = wentLow + 1;
            flagM1 = 1;
        }
        if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
            digitalWrite(m2Pin, LOW);
            wentLow = wentLow + 1;
            flagM2 = 1;
        }
        if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
            digitalWrite(m3Pin, LOW);
            wentLow = wentLow + 1;
            flagM3 = 1;
        }
        if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
            digitalWrite(m4Pin, LOW);
            wentLow = wentLow + 1;
            flagM4 = 1;
        } 
    }
}

static void armMotors() {
    //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
    /*  
     *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
     *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
     *  for other processes that sometimes prevent motors from arming.
     */
    for (int i = 0; i <= 50; i++) {
        commandMotors();
        delay(2);
    }
}


static void throttleCut() {
    //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
    /*
       Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
       minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
       called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
       the motors to anything other than minimum value. Safety first.

       channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
       channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
     */
    if ((channel_5_pwm < 1500) || (armedFly == false)) {
        armedFly = false;
        m1_command_PWM = 120;
        m2_command_PWM = 120;
        m3_command_PWM = 120;
        m4_command_PWM = 120;
    }
}

static void loopRate(int freq) {
    //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
     * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
     * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
     * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
     * and remain above 2kHz, without needing to retune all of our filtering parameters.
     */
    float invFreq = 1.0/freq*1000000.0;
    unsigned long checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}

static void loopBlink() {
    //DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
     * It looks cool.
     */
    if (current_time - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(13, blinkAlternate); //Pin 13 is built in LED

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
    //DESCRIPTION: Simple function to make LED on board blink as desired
    for (int j = 1; j<= numBlinks; j++) {
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

static float invSqrt(float x) {
    return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

/////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(500000); //USB serial
    delay(500);

    //Initialize all pins
    pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 
    pinMode(m1Pin, OUTPUT);
    pinMode(m2Pin, OUTPUT);
    pinMode(m3Pin, OUTPUT);
    pinMode(m4Pin, OUTPUT);

    //Set built in LED to turn on to signal startup
    digitalWrite(13, HIGH);

    delay(5);

    //Initialize radio communication
    Serial1.begin(115000);

    //Set radio channels to default (safe) values before entering main loop
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;

    //Initialize IMU communication
    IMUinit();

    delay(5);

    //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
    //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

    delay(5);

    //Arm OneShot125 motors
    m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
    m2_command_PWM = 125;
    m3_command_PWM = 125;
    m4_command_PWM = 125;

    armMotors(); //Loop over commandMotors() until ESCs happily arm

    //Indicate entering main loop with 3 quick blinks
    setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

    //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
    //calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}


void loop()
{
    //Keep track of what time it is and how much time has elapsed since the last loop
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0;

    loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds
    // Get arming status
    armedStatus(); //Check if the throttle cut is off and throttle is low.

    //Get vehicle state
    getIMUdata(); 

    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); 

    //Compute desired state
    getDesState(); //Convert raw commands to normalized values based on saturated control limits

    //PID Controller - SELECT ONE:
    controlANGLE(); //Stabilize on angle setpoint
                    //controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
                    //controlRATE(); //Stabilize on rate setpoint

                    //Actuator mixing and scaling to PWM values
    controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
    scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

    //Throttle cut check
    throttleCut(); //Directly sets motor commands to low based on state of ch5

    //Command actuators
    commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol

    //Get vehicle commands for next loop iteration
    getCommands(); //Pulls current available radio commands
    failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

    //Regulate loop rate
    loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



