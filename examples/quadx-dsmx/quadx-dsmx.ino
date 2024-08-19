/*
  Hackflight example sketch custom QuadX frame with Spektrum DSMX receiver

  Adapted from https://github.com/nickrehm/dRehmFlight
 
  Copyright (C) 2024 Simon D. Levy
 
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
#include <oneshot125.hpp>

#include "madgwick.hpp"

#include <hackflight.hpp>
#include <utils.hpp>
#include <mixers.hpp>
#include <tasks/debug.hpp>
#include <tasks/blink.hpp>

// Receiver -------------------------------------------------------------------

static Dsm2048 _rx;

void serialEvent2(void)
{
    while (Serial2.available()) {
        _rx.parse(Serial2.read(), micros());
    }
}

static const uint8_t num_DSM_channels = 6;

// IMU ------------------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static const float GYRO_SCALE_FACTOR = 131.0;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static const float ACCEL_SCALE_FACTOR = 16384.0;

static MPU6050 mpu6050;

// Das Blinkenlights ---------------------------------------------------------

static const float    BLINK_RATE_HZ = 1.5;
static hf::BlinkTask _blinkTask;
static const uint8_t LED_PIN = 13;

// Debugging -----------------------------------------------------------------

static const float DEBUG_RATE_HZ = 100;
//static hf::DebugTask _debugTask;
static const auto DEBUG_TASK = hf::DebugTask::DANGLES;

// Motors --------------------------------------------------------------------

static const std::vector<uint8_t> MOTOR_PINS = { 5, 3, 2, 4 };

static auto _motors = OneShot125(MOTOR_PINS);

static uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;


//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
static const unsigned long CHANNEL_1_FS = 1000; //thro
static const unsigned long CHANNEL_2_FS = 1500; //ail
static const unsigned long CHANNEL_3_FS = 1500; //elev
static const unsigned long CHANNEL_4_FS = 1500; //rudd
static const unsigned long CHANNEL_5_FS = 2000; //gear, greater than 1500 = throttle cut
static const unsigned long CHANNEL_6_FS = 2000; //aux1

//Controller parameters (take note of defaults before modifying!): 
static const float I_LIMIT = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)

static const float MAX_PITCH_ROLL = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode

static const float MAX_YAW = 160.0;     //Max yaw rate in deg/sec

static const float KP_PITCH_ROLL_ANGLE = 0.2;    
static const float KI_PITCH_ROLL_ANGLE = 0.3;    
static const float KD_PITCH_ROLL_ANGLE = 0.05;   

// Damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
static const float B_LOOP_PITCH_ROLL = 0.9;      

static const float KP_PITCH_ROLL_RATE = 0.15;    
static const float KI_PITCH_ROLL_RATE = 0.2;     
static const float KD_PITCH_ROLL_RATE = 0.0002;  

static const float KP_YAW = 0.3;           
static const float KI_YAW = 0.05;          
static const float KD_YAW = 0.00015;       

//IMU calibration parameters
static float ACC_ERROR_X = 0.0;
static float ACC_ERROR_Y = 0.0;
static float ACC_ERROR_Z = 0.0;
static float GYRO_ERROR_X = 0.0;
static float GYRO_ERROR_Y= 0.0;
static float GYRO_ERROR_Z = 0.0;


//General stuff
float dt;
unsigned long usec_curr, usec_prev;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;


//IMU:
static float AccX, AccY, AccZ;
static float AccX_prev, AccY_prev, AccZ_prev;
static float GyroX, GyroY, GyroZ;
static float GyroX_prev, GyroY_prev, GyroZ_prev;
static float roll_IMU, pitch_IMU, yaw_IMU;
static float roll_IMU_prev, pitch_IMU_prev;

//Normalized desired state:
static float thro_des, roll_des, pitch_des, yaw_des;
static float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
static float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
static float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
static float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
static float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;

//Flight status
static bool _isArmed;



static void armedStatus() {
    //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
    if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050)) {
        _isArmed = true;
    }
}

static void IMUinit() {
    //DESCRIPTION: Initialize IMU
    /*
     * Don't worry about how this works.
     */

    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

    mpu6050.initialize();

    if (mpu6050.testConnection() == false) {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

}

static void getIMUdata() {

    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    //Accelerometer
    AccX = AcX / ACCEL_SCALE_FACTOR; //G's
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    AccX = AccX - ACC_ERROR_X;
    AccY = AccY - ACC_ERROR_Y;
    AccZ = AccZ - ACC_ERROR_Z;
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
    GyroX = GyroX - GYRO_ERROR_X;
    GyroY = GyroY - GYRO_ERROR_Y;
    GyroZ = GyroZ - GYRO_ERROR_Z;
    //LP filter gyro data
    GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
    GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
    GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
}

static void calculate_IMU_error() 
{
    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note:
    //vehicle should be powered up on flat surface
    /*
     * Don't worry too much about what this is doing. The error values it
     * computes are applied to the raw gyro and accelerometer values AccX,
     * AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift
     * in the
     * measurement. 
     */
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
    ACC_ERROR_X = 0.0;
    ACC_ERROR_Y = 0.0;
    ACC_ERROR_Z = 0.0;
    GYRO_ERROR_X = 0.0;
    GYRO_ERROR_Y= 0.0;
    GYRO_ERROR_Z = 0.0;

    //Read IMU values 12000 times
    int c = 0;
    while (c < 12000) {

        mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

        AccX  = AcX / ACCEL_SCALE_FACTOR;
        AccY  = AcY / ACCEL_SCALE_FACTOR;
        AccZ  = AcZ / ACCEL_SCALE_FACTOR;
        GyroX = GyX / GYRO_SCALE_FACTOR;
        GyroY = GyY / GYRO_SCALE_FACTOR;
        GyroZ = GyZ / GYRO_SCALE_FACTOR;

        //Sum all readings
        ACC_ERROR_X  = ACC_ERROR_X + AccX;
        ACC_ERROR_Y  = ACC_ERROR_Y + AccY;
        ACC_ERROR_Z  = ACC_ERROR_Z + AccZ;
        GYRO_ERROR_X = GYRO_ERROR_X + GyroX;
        GYRO_ERROR_Y = GYRO_ERROR_Y + GyroY;
        GYRO_ERROR_Z = GYRO_ERROR_Z + GyroZ;
        c++;
    }
    //Divide the sum by 12000 to get the error value
    ACC_ERROR_X  = ACC_ERROR_X / c;
    ACC_ERROR_Y  = ACC_ERROR_Y / c;
    ACC_ERROR_Z  = ACC_ERROR_Z / c - 1.0;
    GYRO_ERROR_X = GYRO_ERROR_X / c;
    GYRO_ERROR_Y = GYRO_ERROR_Y / c;
    GYRO_ERROR_Z = GYRO_ERROR_Z / c;

    Serial.print("float ACC_ERROR_X = ");
    Serial.print(ACC_ERROR_X);
    Serial.println(";");
    Serial.print("float ACC_ERROR_Y = ");
    Serial.print(ACC_ERROR_Y);
    Serial.println(";");
    Serial.print("float ACC_ERROR_Z = ");
    Serial.print(ACC_ERROR_Z);
    Serial.println(";");

    Serial.print("float GYRO_ERROR_X = ");
    Serial.print(GYRO_ERROR_X);
    Serial.println(";");
    Serial.print("float GYRO_ERROR_Y = ");
    Serial.print(GYRO_ERROR_Y);
    Serial.println(";");
    Serial.print("float GYRO_ERROR_Z = ");
    Serial.print(GYRO_ERROR_Z);
    Serial.println(";");

    Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

static void calibrateAttitude() {
    //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to
    //converge before commands can be sent to the actuators Assuming vehicle is
    //powered up on level surface!
    /*
     * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
     * to boot. 
     */
    //Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++) {
        usec_prev = usec_curr;      
        usec_curr = micros();      
        dt = (usec_curr - usec_prev)/1000000.0; 
        getIMUdata();
        Madgwick6DOF(dt, GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, roll_IMU, pitch_IMU, yaw_IMU);
        loopRate(2000); //do not exceed 2000Hz
    }
}

static void getDesState() 
{
    thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
    roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
    pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
    yaw_des = -(channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
    roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
    pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
    yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5

    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0)*MAX_PITCH_ROLL; //Between -MAX_PITCH_ROLL and +MAX_PITCH_ROLL
    pitch_des = constrain(pitch_des, -1.0, 1.0)*MAX_PITCH_ROLL; //Between -MAX_PITCH_ROLL and +MAX_PITCH_ROLL
    yaw_des = constrain(yaw_des, -1.0, 1.0)*MAX_YAW; //Between -MAX_YAW and +MAX_YAW
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

static void controlANGLE() {
    //DESCRIPTION: Computes control commands based on state error (angle)
    /*
     * Basic PID control to stablize on angle setpoint based on desired states
     * roll_des, pitch_des, and yaw_des computed in getDesState(). Error is
     * simply the desired state minus the actual state (ex. roll_des -
     * roll_IMU). Two safety features are implimented here regarding the I
     * terms. The I terms are saturated within specified limits on startup to
     * prevent excessive buildup. This can be seen by holding the vehicle at an
     * angle and seeing the motors ramp up on one side until they've maxed out
     * throttle...saturating I to a specified limit fixes this. The second
     * feature defaults the I terms to 0 if the throttle is at the minimum
     * setting. This means the motors will not start spooling up on the ground,
     * and the I terms will always start from 0 on takeoff. This function
     * updates the variables roll_PID, pitch_PID, and yaw_PID which can be
     * thought of as 1-D stablized signals. They are mixed to the configuration
     * of the vehicle in the mixer.
     */

    //Roll
    error_roll = roll_des - roll_IMU;
    integral_roll = integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = GyroX;
    roll_PID = 0.01*(KP_PITCH_ROLL_ANGLE*error_roll + KI_PITCH_ROLL_ANGLE*integral_roll - KD_PITCH_ROLL_ANGLE*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_pitch = GyroY;
    pitch_PID = .01*(KP_PITCH_ROLL_ANGLE*error_pitch + KI_PITCH_ROLL_ANGLE*integral_pitch - KD_PITCH_ROLL_ANGLE*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw, stablize on rate from GyroZ
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(KP_YAW*error_yaw + KI_YAW*integral_yaw + KD_YAW*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    integral_roll_prev = integral_roll;
    //Update pitch variables
    integral_pitch_prev = integral_pitch;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
}

static void controlANGLE2() {
    //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
    /*
     * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
     * See the documentation for tuning this controller.
     */
    //Outer loop - PID on angle
    float roll_des_ol, pitch_des_ol;
    //Roll
    error_roll = roll_des - roll_IMU;
    integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll_ol = 0;
    }
    integral_roll_ol = constrain(integral_roll_ol, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
    roll_des_ol = KP_PITCH_ROLL_ANGLE*error_roll + KI_PITCH_ROLL_ANGLE*integral_roll_ol;// - KD_PITCH_ROLL_ANGLE*derivative_roll;

    //Pitch
    error_pitch = pitch_des - pitch_IMU;
    integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch_ol = 0;
    }
    integral_pitch_ol = constrain(integral_pitch_ol, -I_LIMIT, I_LIMIT); //saturate integrator to prevent unsafe buildup
    derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
    pitch_des_ol = KP_PITCH_ROLL_ANGLE*error_pitch + KI_PITCH_ROLL_ANGLE*integral_pitch_ol;// - KD_PITCH_ROLL_ANGLE*derivative_pitch;

    //Apply loop gain, constrain, and LP filter for artificial damping
    float Kl = 30.0;
    roll_des_ol = Kl*roll_des_ol;
    pitch_des_ol = Kl*pitch_des_ol;
    roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
    pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
    roll_des_ol = (1.0 - B_LOOP_PITCH_ROLL)*roll_des_prev + B_LOOP_PITCH_ROLL*roll_des_ol;
    pitch_des_ol = (1.0 - B_LOOP_PITCH_ROLL)*pitch_des_prev + B_LOOP_PITCH_ROLL*pitch_des_ol;

    //Inner loop - PID on rate
    //Roll
    error_roll = roll_des_ol - GyroX;
    integral_roll_il = integral_roll_prev_il + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll_il = 0;
    }
    integral_roll_il = constrain(integral_roll_il, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = (error_roll - error_roll_prev)/dt; 
    roll_PID = .01*(KP_PITCH_ROLL_RATE*error_roll + KI_PITCH_ROLL_RATE*integral_roll_il + KD_PITCH_ROLL_RATE*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des_ol - GyroY;
    integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch_il = 0;
    }
    integral_pitch_il = constrain(integral_pitch_il, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
    pitch_PID = .01*(KP_PITCH_ROLL_RATE*error_pitch + KI_PITCH_ROLL_RATE*integral_pitch_il + KD_PITCH_ROLL_RATE*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(KP_YAW*error_yaw + KI_YAW*integral_yaw + KD_YAW*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    integral_roll_prev_ol = integral_roll_ol;
    integral_roll_prev_il = integral_roll_il;
    error_roll_prev = error_roll;
    roll_IMU_prev = roll_IMU;
    roll_des_prev = roll_des_ol;
    //Update pitch variables
    integral_pitch_prev_ol = integral_pitch_ol;
    integral_pitch_prev_il = integral_pitch_il;
    error_pitch_prev = error_pitch;
    pitch_IMU_prev = pitch_IMU;
    pitch_des_prev = pitch_des_ol;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;

}

static void controlRATE() {
    //DESCRIPTION: Computes control commands based on state error (rate)
    /*
     * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
     */
    //Roll
    error_roll = roll_des - GyroX;
    integral_roll = integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_roll = (error_roll - error_roll_prev)/dt; 
    roll_PID = .01*(KP_PITCH_ROLL_RATE*error_roll + KI_PITCH_ROLL_RATE*integral_roll + KD_PITCH_ROLL_RATE*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

    //Pitch
    error_pitch = pitch_des - GyroY;
    integral_pitch = integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
    pitch_PID = .01*(KP_PITCH_ROLL_RATE*error_pitch + KI_PITCH_ROLL_RATE*integral_pitch + KD_PITCH_ROLL_RATE*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

    //Yaw, stablize on rate from GyroZ
    error_yaw = yaw_des - GyroZ;
    integral_yaw = integral_yaw_prev + error_yaw*dt;
    if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
        integral_yaw = 0;
    }
    integral_yaw = constrain(integral_yaw, -I_LIMIT, I_LIMIT); //Saturate integrator to prevent unsafe buildup
    derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
    yaw_PID = .01*(KP_YAW*error_yaw + KI_YAW*integral_yaw + KD_YAW*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

    //Update roll variables
    error_roll_prev = error_roll;
    integral_roll_prev = integral_roll;
    GyroX_prev = GyroX;
    //Update pitch variables
    error_pitch_prev = error_pitch;
    integral_pitch_prev = integral_pitch;
    GyroY_prev = GyroY;
    //Update yaw variables
    error_yaw_prev = error_yaw;
    integral_yaw_prev = integral_yaw;
}

static void armMotor(uint8_t & m_usec)
{
    // OneShot125 range from 125 to 250 usec
    m_usec = 125;
}

static uint8_t scaleMotor(const float mval)
{
    return hf::Utils::u8constrain(mval*125 + 125, 125, 250);

}

static void scaleMotors() 
{
    _m1_usec = scaleMotor(m1_command_scaled);
    _m2_usec = scaleMotor(m2_command_scaled);
    _m3_usec = scaleMotor(m3_command_scaled);
    _m4_usec = scaleMotor(m4_command_scaled);
}

static void getCommands() {
    //DESCRIPTION: Get raw PWM values for every channel from the radio
    /*
     * Updates radio PWM commands in loop based on current available commands.
     * channel_x_pwm is the raw command used in the rest of the loop. If using
     * a PWM or PPM receiver, the radio commands are retrieved from a function
     * in the readPWM file separate from this one which is running a bunch of
     * interrupts to continuously update the radio readings. If using an SBUS
     * receiver, the alues are pulled from the SBUS library directly.  The raw
     * radio commands are filtered with a first order low-pass filter to
     * eliminate any really high frequency noise. 
     */

    if (_rx.timedOut(micros())) {
        //Serial.println("*** DSM RX TIMED OUT ***");
    }
    else if (_rx.gotNewFrame()) {
        uint16_t values[num_DSM_channels];
        _rx.getChannelValues(values, num_DSM_channels);

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
        channel_1_pwm = CHANNEL_1_FS;
        channel_2_pwm = CHANNEL_2_FS;
        channel_3_pwm = CHANNEL_3_FS;
        channel_4_pwm = CHANNEL_4_FS;
        channel_5_pwm = CHANNEL_5_FS;
        channel_6_pwm = CHANNEL_6_FS;
    }
}

static void runMotors() 
{
    _motors.set(0, _m1_usec);
    _motors.set(1, _m2_usec);
    _motors.set(2, _m3_usec);
    _motors.set(3, _m4_usec);

    _motors.run();
}

static void cutMotors() 
{
    if ((channel_5_pwm < 1500) || (_isArmed == false)) {
        _isArmed = false;
        _m1_usec = 120;
        _m2_usec = 120;
        _m3_usec = 120;
        _m4_usec = 120;

    }
}

static void loopRate(int freq) 
{
    //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain
     * stable and whatnot. Interrupt routines running in the background cause
     * the loop rate to fluctuate. This function basically just waits at the
     * end of every loop iteration until the correct time has passed since the
     * start of the current loop for the desired loop rate in Hz. 2kHz is a
     * good rate to be at because the loop nominally will run between 2.8kHz -
     * 4.2kHz. This lets us have a little room to add extra computations and
     * remain above 2kHz, without needing to retune all of our filtering
     * parameters.
     */
    float invFreq = 1.0/freq*1000000.0;
    unsigned long checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - usec_curr)) {
        checker = micros();
    }
}


static void printRadioData() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F(" CH1:"));
        Serial.print(channel_1_pwm);
        Serial.print(F(" CH2:"));
        Serial.print(channel_2_pwm);
        Serial.print(F(" CH3:"));
        Serial.print(channel_3_pwm);
        Serial.print(F(" CH4:"));
        Serial.print(channel_4_pwm);
        Serial.print(F(" CH5:"));
        Serial.print(channel_5_pwm);
        Serial.print(F(" CH6:"));
        Serial.print(channel_6_pwm);
        Serial.print(F(" Armed:"));
        Serial.println(_isArmed);
    }
}

static void printDesiredState() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("thro_des:"));
        Serial.print(thro_des);
        Serial.print(F(" roll_des:"));
        Serial.print(roll_des);
        Serial.print(F(" pitch_des:"));
        Serial.print(pitch_des);
        Serial.print(F(" yaw_des:"));
        Serial.println(yaw_des);
    }
}

static void printGyroData() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("GyroX:"));
        Serial.print(GyroX);
        Serial.print(F(" GyroY:"));
        Serial.print(GyroY);
        Serial.print(F(" GyroZ:"));
        Serial.println(GyroZ);
    }
}

static void printAccelData() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("AccX:"));
        Serial.print(AccX);
        Serial.print(F(" AccY:"));
        Serial.print(AccY);
        Serial.print(F(" AccZ:"));
        Serial.println(AccZ);
    }
}

static void printRollPitchYaw() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("roll:"));
        Serial.print(roll_IMU);
        Serial.print(F(" pitch:"));
        Serial.print(pitch_IMU);
        Serial.print(F(" yaw:"));
        Serial.println(yaw_IMU);
    }
}

static void printPIDoutput() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("roll_PID:"));
        Serial.print(roll_PID);
        Serial.print(F(" pitch_PID:"));
        Serial.print(pitch_PID);
        Serial.print(F(" yaw_PID:"));
        Serial.println(yaw_PID);
    }
}

static void printMotorCommands() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("m1_command:"));
        Serial.print(_m1_usec);
        Serial.print(F(" m2_command:"));
        Serial.print(_m2_usec);
        Serial.print(F(" m3_command:"));
        Serial.print(_m3_usec);
        Serial.print(F(" m4_command:"));
        Serial.println(_m4_usec);
    }
}

static void printLoopRate() {
    if (usec_curr - print_counter > 10000) {
        print_counter = micros();
        Serial.print(F("dt:"));
        Serial.println(dt*1000000.0);
    }
}

static void blinkOnStartup(void)
{
    // These constants are arbitrary, so we hide them here
    static const uint8_t  BLINK_INIT_COUNT = 10;
    static const uint32_t BLINK_INIT_TIME_MSEC = 100;

    for (uint8_t j=0; j<BLINK_INIT_COUNT; j++) {
        digitalWrite(LED_PIN, LOW);
        delay(BLINK_INIT_TIME_MSEC);
        digitalWrite(LED_PIN, HIGH);
        delay(BLINK_INIT_TIME_MSEC);
    }
}
//////////////////////////////////////////////////////////////////////////////

void setup() {

    (void)calibrateAttitude;
    (void)calculate_IMU_error;

    Serial.begin(500000); //USB serial
    delay(500);

    Serial2.begin(115200);

    //Initialize all pins
    pinMode(LED_PIN, OUTPUT); 

    //Set LED to turn on to signal startup
    digitalWrite(LED_PIN, HIGH);

    delay(5);

    //Set radio channels to default (safe) values before entering main loop
    channel_1_pwm = CHANNEL_1_FS;
    channel_2_pwm = CHANNEL_2_FS;
    channel_3_pwm = CHANNEL_3_FS;
    channel_4_pwm = CHANNEL_4_FS;
    channel_5_pwm = CHANNEL_5_FS;
    channel_6_pwm = CHANNEL_6_FS;

    //Initialize IMU communication
    IMUinit();

    delay(5);

    //Get IMU error to zero accelerometer and gyro readings, assuming vehicle
    //is level when powered up calculate_IMU_error(); //Calibration parameters
    //printed to serial monitor. Paste these in the user specified variables
    //section, then comment this out forever.

    delay(5);

    //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by
    //setting throttle stick to max, powering on, and lowering throttle to zero
    //after the beeps Code will not proceed past here if this function is
    //uncommented!

    // Arm OneShot125 motors
    armMotor(_m1_usec);
    armMotor(_m2_usec);
    armMotor(_m3_usec);
    armMotor(_m4_usec);
    _motors.arm();

    //Indicate entering main loop with some quick blinks
    blinkOnStartup(); 
}

void loop() 
{
    //Keep track of what time it is and how much time has elapsed since the last loop
    usec_prev = usec_curr;      
    usec_curr = micros();      
    dt = (usec_curr - usec_prev)/1000000.0;

    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //printRadioData();     
    //printDesiredState();  
    //printGyroData();     
    //printAccelData();    
    //printRollPitchYaw(); 
    //printPIDoutput();    
    //printMotorCommands();
    //printLoopRate();     

    (void)printRadioData;     
    (void)printDesiredState;  
    (void)printGyroData;      
    (void)printAccelData;     
    (void)printRollPitchYaw;  
    (void)printPIDoutput;     
    (void)printMotorCommands; 
    (void)printLoopRate;      

    // Get arming status
    armedStatus(); //Check if the throttle cut is off and throttle is low.

    // LED should be on when armed
    if (_isArmed) {
        digitalWrite(LED_PIN, HIGH);
    }

    // Otherwise, blink LED as heartbeat
    else {
        _blinkTask.run(usec_curr, BLINK_RATE_HZ);
    }


    //Get vehicle state
    getIMUdata(); 
    Madgwick6DOF(dt, GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, roll_IMU, pitch_IMU, yaw_IMU);

    //Compute desired state
    getDesState(); //Convert raw commands to normalized values based on saturated control limits

    //PID Controller - SELECT ONE:
    controlANGLE(); //Stabilize on angle setpoint
    (void)controlANGLE2; //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
    (void)controlRATE; //Stabilize on rate setpoint

    // Run motor mixer
    hf::Mixer::runDF(thro_des, roll_PID, pitch_PID, yaw_PID, 
            m1_command_scaled, 
            m2_command_scaled, 
            m3_command_scaled, 
            m4_command_scaled);

    // Rescale motor values for OneShot125
    scaleMotors(); 

    // Turn off motors under various conditions
    cutMotors(); 

    // Run motors
    runMotors(); 

    //Get vehicle commands for next loop iteration
    getCommands(); //Pulls current available radio commands
    failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

    //Regulate loop rate
    loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}
