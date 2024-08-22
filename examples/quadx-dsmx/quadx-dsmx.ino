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

// PID control ---------------------------------------------------------------

static hf::AnglePid _anglePid;

// Das Blinkenlights ---------------------------------------------------------

static const float BLINK_RATE_HZ = 1.5;
static const uint8_t LED_PIN = 13;
static hf::BlinkTask _blinkTask;

// Motors --------------------------------------------------------------------

static const std::vector<uint8_t> MOTOR_PINS = { 5, 3, 2, 4 };

static auto _motors = OneShot125(MOTOR_PINS);

static uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;

//Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
static const float MAX_PITCH_ROLL_ANGLE = 30.0;    

static const float MAX_YAW_RATE = 160.0;     //Max yaw rate in deg/sec

// IMU calibration parameters
static float ACC_ERROR_X = 0.0;
static float ACC_ERROR_Y = 0.0;
static float ACC_ERROR_Z = 0.0;
static float GYRO_ERROR_X = 0.0;
static float GYRO_ERROR_Y= 0.0;
static float GYRO_ERROR_Z = 0.0;

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

static void readImu(
        float & AccX, float & AccY, float & AccZ,
        float & GyroX, float & GyroY, float & GyroZ
        ) 
{
    static float AccX_prev, AccY_prev, AccZ_prev;
    static float GyroX_prev, GyroY_prev, GyroZ_prev;

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

    // Negate GyroZ for nose-right positive
    GyroZ = -GyroZ;
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

static void readReceiver(
        uint32_t & chan_1,
        uint32_t & chan_2,
        uint32_t & chan_3,
        uint32_t & chan_4,
        uint32_t & chan_5,
        uint32_t & chan_6,
        bool & isArmed, 
        bool & gotFailsafe) 
{
    if (_rx.timedOut(micros())) {
        isArmed = false;
        gotFailsafe = true;
    }

    else if (_rx.gotNewFrame()) {
        uint16_t values[NUM_DSM_CHANNELS];
        _rx.getChannelValues(values, NUM_DSM_CHANNELS);

        chan_1 = values[0];
        chan_2 = values[1];
        chan_3 = values[2];
        chan_4 = values[3];
        chan_5 = values[4];
        chan_6 = values[5];
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

static void cutMotors(const uint32_t chan_5, bool & isArmed) 
{
    if (chan_5 < 1500 || !isArmed) {
        isArmed = false;
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
    // Safety
    static bool _isArmed;
    static bool _gotFailsafe;

    static uint32_t chan_1, chan_2, chan_3, chan_4, chan_5, chan_6;

    // Keep track of what time it is and how much time has elapsed since the last loop
    const auto usec_curr = micros();      
    static uint32_t _usec_prev;
    const float dt = (usec_curr - _usec_prev)/1000000.0;
    _usec_prev = usec_curr;      

    // Arm vehicle if safe
    if (!_gotFailsafe && (chan_5 > 1500) && (chan_1 < 1050)) {
        _isArmed = true;
    }

    // LED should be on when armed
    if (_isArmed) {
        digitalWrite(LED_PIN, HIGH);
    }

    // Otherwise, blink LED as heartbeat
    else {
        _blinkTask.run(usec_curr, BLINK_RATE_HZ);
    }

    //Get vehicle state

    float AccX = 0, AccY = 0, AccZ = 0;
    float GyroX = 0, GyroY = 0, GyroZ = 0;

    readImu(AccX, AccY, AccZ, GyroX, GyroY, GyroZ); 

    // Get Euler angles from IMU (note negations)
    float phi = 0, theta = 0, psi = 0;
    Madgwick6DOF(dt, GyroX, -GyroY, GyroZ, -AccX, AccY, AccZ, phi, theta, psi);

    // Convert stick demands to appropriate intervals
    const float thro_demand =
        constrain((chan_1 - 1000.0) / 1000.0, 0.0, 1.0);
    const float roll_demand = 
        constrain((chan_2 - 1500.0) / 500.0, -1.0, 1.0) * MAX_PITCH_ROLL_ANGLE;
    const float pitch_demand =
        constrain((chan_3 - 1500.0) / 500.0, -1.0, 1.0) * MAX_PITCH_ROLL_ANGLE;
    const float yaw_demand = 
        constrain((chan_4 - 1500.0) / 500.0, -1.0, 1.0) * MAX_YAW_RATE;

    // Run demands through PID controller
    float roll_PID=0, pitch_PID=0, yaw_PID=0;
    _anglePid.run(dt, thro_demand, 
            roll_demand, pitch_demand, yaw_demand, 
            phi, theta,
            GyroX, GyroY, GyroZ,
            roll_PID, pitch_PID, yaw_PID);

    float m1_command=0, m2_command=0, m3_command=0, m4_command=0;

    // Run motor mixer
    hf::Mixer::runDF(thro_demand, roll_PID, pitch_PID, yaw_PID, 
            m1_command, m2_command, m3_command, m4_command);

    // Rescale motor values for OneShot125
    _m1_usec = scaleMotor(m1_command);
    _m2_usec = scaleMotor(m2_command);
    _m3_usec = scaleMotor(m3_command);
    _m4_usec = scaleMotor(m4_command);

    // Turn off motors under various conditions
    cutMotors(chan_5, _isArmed); 

    // Run motors
    runMotors(); 

    // Get vehicle commands for next loop iteration
    readReceiver(chan_1, chan_2, chan_3, chan_4, chan_5, chan_6,
            _isArmed, _gotFailsafe); 

    // Regulate loop rate: Do not exceed 2000Hz, all filter parameters tuned to
    // 2000Hz by default
    runLoopDelay(usec_curr, 2000); 
}
