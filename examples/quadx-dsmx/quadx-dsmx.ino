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
#include <pids/angle.hpp>

// Receiver -------------------------------------------------------------------

static Dsm2048 _rx;

void serialEvent2(void)
{
    while (Serial2.available()) {
        _rx.parse(Serial2.read(), micros());
    }
}

static const uint8_t NUM_DSM_CHANNELS = 6;

// IMU ------------------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static const float GYRO_SCALE_FACTOR = 131.0;

static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static const float ACCEL_SCALE_FACTOR = 16384.0;

static MPU6050 _mpu6050;

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
static unsigned long usec_curr;
static unsigned long print_counter;

//Radio communication:
uint32_t channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
uint32_t channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

// IMU
static float AccX, AccY, AccZ;
static float AccX_prev, AccY_prev, AccZ_prev;
static float GyroX, GyroY, GyroZ;
static float GyroX_prev, GyroY_prev, GyroZ_prev;
static float roll_IMU, pitch_IMU, yaw_IMU;

// Normalized desired state:
static float thro_des, roll_des, pitch_des, yaw_des;

// Controller:
static float roll_PID;
static float pitch_PID; 
static float yaw_PID;

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

static void initImu() 
{
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

    _mpu6050.initialize();

    if (!_mpu6050.testConnection()) {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while(true) {}
    }

    // From the reset state all registers should be 0x00, so we should be at
    // max sample rate with digital low pass filter(s) off.  All we need to
    // do is set the desired fullscale ranges
    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

}

static void getIMUdata() {

    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

    _mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    //Accelerometer
    AccX = AcX / ACCEL_SCALE_FACTOR; //G's
    AccY = AcY / ACCEL_SCALE_FACTOR;
    AccZ = AcZ / ACCEL_SCALE_FACTOR;

    // Correct the outputs with the calculated error values
    AccX = AccX - ACC_ERROR_X;
    AccY = AccY - ACC_ERROR_Y;
    AccZ = AccZ - ACC_ERROR_Z;

    // LP filter accelerometer data
    AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
    AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
    AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    // Gyro
    GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;

    // Correct the outputs with the calculated error values
    GyroX = GyroX - GYRO_ERROR_X;
    GyroY = GyroY - GYRO_ERROR_Y;
    GyroZ = GyroZ - GYRO_ERROR_Z;

    // LP filter gyro data
    GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
    GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
    GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
}

static void getDesState() 
{
    thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
    roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
    pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
    yaw_des = -(channel_4_pwm - 1500.0)/500.0; //Between -1 and 1

    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0)*MAX_PITCH_ROLL; //Between -MAX_PITCH_ROLL and +MAX_PITCH_ROLL
    pitch_des = constrain(pitch_des, -1.0, 1.0)*MAX_PITCH_ROLL; //Between -MAX_PITCH_ROLL and +MAX_PITCH_ROLL
    yaw_des = constrain(yaw_des, -1.0, 1.0)*MAX_YAW; //Between -MAX_YAW and +MAX_YAW
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
        uint16_t values[NUM_DSM_CHANNELS];
        _rx.getChannelValues(values, NUM_DSM_CHANNELS);

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

static void runLoopDelay(const uint32_t usec_curr, const uint32_t freq) 
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

static void printLoopRate(const float dt) {
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

    // Set up serial debugging
    Serial.begin(500000);
    delay(500);

    // Set up receiver UART
    Serial2.begin(115200);

    // Initialize LED
    pinMode(LED_PIN, OUTPUT); 
    digitalWrite(LED_PIN, HIGH);

    delay(5);

    // Set radio channels to default (safe) values before entering main loop
    channel_1_pwm = CHANNEL_1_FS;
    channel_2_pwm = CHANNEL_2_FS;
    channel_3_pwm = CHANNEL_3_FS;
    channel_4_pwm = CHANNEL_4_FS;
    channel_5_pwm = CHANNEL_5_FS;
    channel_6_pwm = CHANNEL_6_FS;

    // Initialize IMU communication
    initImu();

    delay(5);

    // Arm OneShot125 motors
    armMotor(_m1_usec);
    armMotor(_m2_usec);
    armMotor(_m3_usec);
    armMotor(_m4_usec);
    _motors.arm();

    // Indicate entering main loop with some quick blinks
    blinkOnStartup(); 
}

void loop() 
{
    //Keep track of what time it is and how much time has elapsed since the last loop
    static uint32_t usec_prev;
    usec_prev = usec_curr;      
    usec_curr = micros();      
    const float dt = (usec_curr - usec_prev)/1000000.0;

    //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    //printRadioData();     
    //printDesiredState();  
    //printGyroData();     
    //printAccelData();    
    //printRollPitchYaw(); 
    //printPIDoutput();    
    //printMotorCommands();
    //printLoopRate(dt);     

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

    hf::AnglePid::run(
            dt, 
            roll_des, 
            pitch_des, 
            yaw_des, 
            roll_IMU,
            pitch_IMU,
            channel_1_pwm,
            GyroX,
            GyroY,
            GyroZ,
            roll_PID,
            pitch_PID,
            yaw_PID);


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

    // Regulate loop rate: Do not exceed 2000Hz, all filter parameters tuned to
    // 2000Hz by default
    runLoopDelay(usec_curr, 2000); 
}
