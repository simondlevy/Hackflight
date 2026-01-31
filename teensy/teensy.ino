#include <Wire.h> 

#include <dsmrx.hpp>  
#include <MPU6050.h>

#define GYRO_250DPS 

#define ACCEL_2G 

static MPU6050 _mpu6050;

static Dsm2048 _dsm2048;

void serialEvent1(void)
{
    while (Serial1.available()) {
        _dsm2048.parse(Serial1.read(), micros());
    }
}

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

//

//                           

//

static unsigned long channel_1_fs = 1000; 

static unsigned long channel_2_fs = 1500; 

static unsigned long channel_3_fs = 1500; 

static unsigned long channel_4_fs = 1500; 

static unsigned long channel_5_fs = 2000; 

static unsigned long channel_6_fs = 2000; 

static float B_madgwick = 0.04;  

static float B_accel = 0.14;     

static float B_gyro = 0.1;       

static float AccErrorX = 0.0;
static float AccErrorY = 0.0;
static float AccErrorZ = 0.0;
static float GyroErrorX = 0.0;
static float GyroErrorY= 0.0;
static float GyroErrorZ = 0.0;

static float i_limit = 25.0;     

static float maxRoll = 30.0;     

static float maxPitch = 30.0;    

static float maxYaw = 160.0;     

static float Kp_roll_angle = 0.2;    

static float Ki_roll_angle = 0.3;    

static float Kd_roll_angle = 0.05;   

static float Kp_pitch_angle = 0.2;   

static float Ki_pitch_angle = 0.3;   

static float Kd_pitch_angle = 0.05;  

static float Kp_yaw = 0.3;           

static float Ki_yaw = 0.05;          

static float Kd_yaw = 0.00015;       

static const int m1Pin = 6;
static const int m2Pin = 5;
static const int m3Pin = 4;
static const int m4Pin = 3;

static float dt;
static unsigned long current_time, prev_time;
static unsigned long blink_counter, blink_delay;
static bool blinkAlternate;

static unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
static unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

static float AccX, AccY, AccZ;
static float AccX_prev, AccY_prev, AccZ_prev;
static float GyroX, GyroY, GyroZ;
static float GyroX_prev, GyroY_prev, GyroZ_prev;
static float roll_IMU, pitch_IMU, yaw_IMU;

static float q0 = 1.0f; 

static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

static float thro_des, roll_des, pitch_des, yaw_des;
static float roll_passthru, pitch_passthru, yaw_passthru;

static float error_roll, integral_roll, integral_roll_prev, derivative_roll,
             roll_PID;

static float error_pitch, integral_pitch, integral_pitch_prev,
             derivative_pitch, pitch_PID;

static float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev,
             derivative_yaw, yaw_PID;

static float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;

static int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;

static bool armedFly;

static void controlMixer() {

    m1_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; 

    m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; 

    m3_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; 

    m4_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; 

}

static void armedStatus() {

    if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050)) {
        armedFly = true;
    }
}

static void IMUinit() {

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

static void getIMUdata() {

    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

    _mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    AccX = AcX / ACCEL_SCALE_FACTOR; 

    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;

    AccX = AccX - AccErrorX;
    AccY = AccY - AccErrorY;
    AccZ = AccZ - AccErrorZ;

    AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
    AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
    AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    GyroX = GyX / GYRO_SCALE_FACTOR; 

    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;

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

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

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

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 

        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= B_madgwick * s0;
        qDot2 -= B_madgwick * s1;
        qDot3 -= B_madgwick * s2;
        qDot4 -= B_madgwick * s3;
    }

    q0 += qDot1 * invSampleFreq;
    q1 += qDot2 * invSampleFreq;
    q2 += qDot3 * invSampleFreq;
    q3 += qDot4 * invSampleFreq;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; 

    pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; 

    yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; 

}

static void getDesState() {

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

static void controlANGLE() {

    error_roll = roll_des - roll_IMU;
    integral_roll = integral_roll_prev + error_roll*dt;
    if (channel_1_pwm < 1060) {   

        integral_roll = 0;
    }
    integral_roll = constrain(integral_roll, -i_limit, i_limit); 

    derivative_roll = GyroX;
    roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); 

    error_pitch = pitch_des - pitch_IMU;
    integral_pitch = integral_pitch_prev + error_pitch*dt;
    if (channel_1_pwm < 1060) {   

        integral_pitch = 0;
    }
    integral_pitch = constrain(integral_pitch, -i_limit, i_limit); 

    derivative_pitch = GyroY;
    pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); 

    error_yaw = -yaw_des - GyroZ;
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

static void scaleCommands() {

    m1_command_PWM = m1_command_scaled*125 + 125;
    m2_command_PWM = m2_command_scaled*125 + 125;
    m3_command_PWM = m3_command_scaled*125 + 125;
    m4_command_PWM = m4_command_scaled*125 + 125;

    m1_command_PWM = constrain(m1_command_PWM, 125, 250);
    m2_command_PWM = constrain(m2_command_PWM, 125, 250);
    m3_command_PWM = constrain(m3_command_PWM, 125, 250);
    m4_command_PWM = constrain(m4_command_PWM, 125, 250);
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

    float b = 0.7; 

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

    unsigned minVal = 800;
    unsigned maxVal = 2200;
    int check1 = 0;
    int check2 = 0;
    int check3 = 0;
    int check4 = 0;
    int check5 = 0;
    int check6 = 0;

    if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
    if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
    if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
    if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
    if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
    if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

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

    int wentLow = 0;
    int pulseStart, timer;
    int flagM1 = 0;
    int flagM2 = 0;
    int flagM3 = 0;
    int flagM4 = 0;

    digitalWrite(m1Pin, HIGH);
    digitalWrite(m2Pin, HIGH);
    digitalWrite(m3Pin, HIGH);
    digitalWrite(m4Pin, HIGH);
    pulseStart = micros();

    while (wentLow < 4 ) { 

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

    for (int i = 0; i <= 50; i++) {
        commandMotors();
        delay(2);
    }
}

static void throttleCut() {

    if ((channel_5_pwm < 1500) || (armedFly == false)) {
        armedFly = false;
        m1_command_PWM = 120;
        m2_command_PWM = 120;
        m3_command_PWM = 120;
        m4_command_PWM = 120;
    }
}

static void loopRate(int freq) {

    float invFreq = 1.0/freq*1000000.0;
    unsigned long checker = micros();

    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}

static void loopBlink() {

    if (current_time - blink_counter > blink_delay) {
        blink_counter = micros();
        digitalWrite(13, blinkAlternate); 

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
        digitalWrite(13, LOW);
        delay(downTime);
        digitalWrite(13, HIGH);
        delay(upTime);
    }
}

static float invSqrt(float x) {
    return 1.0/sqrtf(x); 

}

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
///////////////////////////////////////////////////////
//////////////////////////////////////////////////////
/////////////////////////////////////////////////////
////////////////////////////////////////////////////
///////////////////////////////////////////////////
//////////////////////////////////////////////////
/////////////////////////////////////////////////
////////////////////////////////////////////////
///////////////////////////////////////////////
//////////////////////////////////////////////
/////////////////////////////////////////////
////////////////////////////////////////////
///////////////////////////////////////////
//////////////////////////////////////////
/////////////////////////////////////////
////////////////////////////////////////
///////////////////////////////////////
//////////////////////////////////////
/////////////////////////////////////
////////////////////////////////////
///////////////////////////////////
//////////////////////////////////
/////////////////////////////////
////////////////////////////////
///////////////////////////////
//////////////////////////////
/////////////////////////////
////////////////////////////
///////////////////////////
//////////////////////////
/////////////////////////
////////////////////////
///////////////////////
//////////////////////
/////////////////////
////////////////////
///////////////////
//////////////////
/////////////////
////////////////
///////////////
//////////////
/////////////
////////////
///////////
//////////
/////////
////////
///////
//////
/////
////
///
//

void setup()
{
    Serial.begin(500000); 

    delay(500);

    pinMode(13, OUTPUT); 

    pinMode(m1Pin, OUTPUT);
    pinMode(m2Pin, OUTPUT);
    pinMode(m3Pin, OUTPUT);
    pinMode(m4Pin, OUTPUT);

    digitalWrite(13, HIGH);

    delay(5);

    Serial1.begin(115000);

    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;

    IMUinit();

    delay(5);

//Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

    delay(5);

    m1_command_PWM = 125; 

    m2_command_PWM = 125;
    m3_command_PWM = 125;
    m4_command_PWM = 125;

    armMotors(); 

    setupBlink(3,160,70); 

//Generates magentometer error and scale factors to be pasted in user-specified variables section

}

void loop()
{

    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0;

    loopBlink(); 

    armedStatus(); 

    getIMUdata(); 

    Madgwick6DOF(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); 

    getDesState(); 

    controlANGLE(); 

//Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!

//Stabilize on rate setpoint

    controlMixer(); 

    scaleCommands(); 

    throttleCut(); 

    commandMotors(); 

    getCommands(); 

    failSafe(); 

    loopRate(2000); 

}


