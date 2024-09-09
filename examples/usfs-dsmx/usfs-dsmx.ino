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

#include <usfs.hpp>

#include <dsmrx.hpp>
#include <oneshot125.hpp>

#include <hackflight.hpp>
#include <utils.hpp>
#include <mixers.hpp>
#include <tasks/blink.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/yaw_rate.hpp>

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

static const uint8_t ACCEL_BANDWIDTH = 3;
static const uint8_t GYRO_BANDWIDTH  = 3;
static const uint8_t QUAT_DIVISOR    = 1;
static const uint8_t MAG_RATE        = 100;
static const uint8_t ACCEL_RATE      = 20; // Multiply by 10 to get actual rate
static const uint8_t GYRO_RATE       = 100; // Multiply by 10 to get actual rate
static const uint8_t BARO_RATE       = 50;

static const uint8_t INTERRUPT_ENABLE = 
Usfs::INTERRUPT_RESET_REQUIRED |
Usfs::INTERRUPT_ERROR |
Usfs::INTERRUPT_QUAT;

static const bool VERBOSE = false;

static Usfs _usfs;

// PID control ---------------------------------------------------------------

static constexpr float THROTTLE_DOWN = 0.06;

static hf::YawRatePid _yawRatePid;

static hf::PitchRollAnglePid _pitchRollAnglePid;
static hf::PitchRollRatePid _pitchRollRatePid;

// Das Blinkenlights ---------------------------------------------------------

static const float BLINK_RATE_HZ = 1.5;
static const uint8_t LED_PIN = 0;
static hf::BlinkTask _blinkTask;

// Motors --------------------------------------------------------------------

static const std::vector<uint8_t> MOTOR_PINS = { 2, 3, 4, 5 };

static auto _motors = OneShot125(MOTOR_PINS);

static uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec;

// Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
static const float PITCH_ROLL_PRESCALE = 30.0;    

// Max yaw rate in deg/sec
static const float YAW_PRESCALE = 160.0;     

static void initImu() 
{
    Wire.begin();
    Wire.setClock(400000); 

    _usfs.loadFirmware(VERBOSE); 

    _usfs.begin(
            ACCEL_BANDWIDTH,
            GYRO_BANDWIDTH,
            QUAT_DIVISOR,
            MAG_RATE,
            ACCEL_RATE,
            GYRO_RATE,
            BARO_RATE,
            INTERRUPT_ENABLE,
            VERBOSE); 
}

static void readImu(
        float & phi, float & theta, float & psi,
        float & gyroX, float & gyroY, float & gyroZ
        ) 
{
    uint8_t eventStatus = Usfs::checkStatus(); 

    // Keep these around between reads
    static float _gyroX, _gyroY, _gyroZ;
    static float _phi, _theta, _psi;

    if (Usfs::eventStatusIsError(eventStatus)) { 

        Usfs::reportError(eventStatus);
    }

    if (Usfs::eventStatusIsGyrometer(eventStatus)) { 

        _usfs.readGyrometerScaled(_gyroX, _gyroY, _gyroZ);
    }

    if (Usfs::eventStatusIsQuaternion(eventStatus)) { 

        float qw=0, qx=0, qy=0, qz=0;

        _usfs.readQuaternion(qw, qx, qy, qz);

        hf::Utils::quat2euler(qw, qx, qy, qz, _phi, _theta, _psi);
    }

    phi = _phi;
    theta = -_theta; // Negate for nose-down positive
    psi = -_psi; // Negate for nose-righ positive

    gyroX = _gyroX;
    gyroY = _gyroY;
    gyroZ = _gyroZ;
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
    armMotor(_m4_usec);
    armMotor(_m2_usec);
    armMotor(_m1_usec);
    armMotor(_m3_usec);
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
        _blinkTask.run(LED_PIN, usec_curr, BLINK_RATE_HZ);
    }

    // Get Euler angles and angular rates (gyro) from IMU

    float phi = 0, theta = 0, psi = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;

    // Note gyro X/Y swap
    readImu(phi, theta, psi, gyroY, gyroX, gyroZ); 

    static uint32_t msec_prev;
    const auto msec_curr = millis();
    if (msec_curr - msec_prev > 100) {
        printf("phi:%f\n", phi);
        msec_prev = msec_curr;
    }

    // Convert stick demands to appropriate intervals
    float thrustDemand =
        constrain((chan_1 - 1000.0) / 1000.0, 0.0, 1.0);
    float rollDemand = 
        constrain((chan_2 - 1500.0) / 500.0, -1.0, 1.0) * PITCH_ROLL_PRESCALE;
    float pitchDemand =
        constrain((chan_3 - 1500.0) / 500.0, -1.0, 1.0) * PITCH_ROLL_PRESCALE;
    float yawDemand = 
        constrain((chan_4 - 1500.0) / 500.0, -1.0, 1.0) * YAW_PRESCALE;

    const auto resetPids = thrustDemand < THROTTLE_DOWN;

    // Run demands through PID controllers

    _pitchRollAnglePid.run(
            dt, resetPids, rollDemand, pitchDemand, phi, theta);

    _pitchRollRatePid.run(
            dt, resetPids, rollDemand, pitchDemand, gyroX, gyroY);

    _yawRatePid.run(dt, resetPids, yawDemand, gyroZ);

    float m1_command=0, m2_command=0, m3_command=0, m4_command=0;

    // Run motor mixer
    hf::Mixer::runBetaFlightQuadX(
            thrustDemand, rollDemand, pitchDemand, yawDemand, 
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
