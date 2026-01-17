/*
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

// Hackflight library
#include <hackflight.h>
#include <datatypes.h>
#include <firmware/estimators/ekf.hpp>
#include <firmware/estimators/madgwick.hpp>
#include <firmware/imu.hpp>
#include <firmware/timer.hpp>

// FAFO -----------------------------------------------------------
static const uint32_t LOOP_FREQ_HZ = 2000;

// MPU6050 ------------------------------------------------------------

static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_250;
static constexpr float GYRO_SCALE_FACTOR = 131.0;
static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_2;
static constexpr float ACCEL_SCALE_FACTOR = 16384.0;
static MPU6050 _mpu6050 = MPU6050(0x68, &Wire1);
static MadgwickFilter  _madgwick;

// BMI088 ------------------------------------------------------------

static const uint32_t FREQ_EKF_PREDICTION = 100;
static IMU _imu;
static EKF _ekf;

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

static void reportForever(const char * message)
{
    while (true) {
        printf("%s\n", message);
        delay(500);
    }
}

static void initMpu6050() 
{
    //Note this is 2.5 times the spec sheet 400 kHz max...
    Wire1.setClock(1000000); 

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


float getDt()
{
    const auto usec_curr = micros();      
    static uint32_t _usec_prev;
    const auto dt = (usec_curr - _usec_prev)/1000000.0;
    _usec_prev = usec_curr;      
    return dt;
}

static void getMadgwickState(const float dt, vehicleState_t & state)
{
    // Read IMU
    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
    _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Accelerometer Gs
    const axis3_t accel = {
        -ax / ACCEL_SCALE_FACTOR,
        ay / ACCEL_SCALE_FACTOR,
        az / ACCEL_SCALE_FACTOR
    };

    // Gyro deg /sec
    const axis3_t gyro = {
        gx / GYRO_SCALE_FACTOR, 
        -gy / GYRO_SCALE_FACTOR,
        -gz / GYRO_SCALE_FACTOR
    };

    // Run state estimator to get Euler angles
    _madgwick.getEulerAngles(dt, gyro, accel,
         state.phi, state.theta, state.psi);

    // Get angular velocities directly from gyro
    state.dphi = gyro.x;
    state.dtheta = -gyro.y;
    state.dpsi = gyro.z;
}

static void getEkfState(const uint32_t msec, vehicleState_t & estate)
{
    static Timer _timer;

    if (_timer.ready(FREQ_EKF_PREDICTION)) {
        _ekf.predict(msec, true); 
    }

    _imu.step(&_ekf, msec);
    _ekf.getStateEstimate(msec, estate);
}

static void dump(
      const uint32_t msec,
      const vehicleState_t & mstate,
      const vehicleState_t & estate)
{
    static uint32_t _msec;
    if (msec - _msec > 10) {
       printf("m: phi=%+3.0f theta=%+3.0f psi=%+3.0f | "
              "e: phi=%+3.0f theta=%+3.0f psi=%+3.0f\n ", 
              mstate.phi, mstate.theta, mstate.psi,
              estate.phi, estate.theta, estate.psi);
        _msec = msec;
    }
}


//////////////////////////////////////////////////////////////////////////////

void setup() 
{
    Wire1.begin();
    initMpu6050();

    _imu.init();
    _ekf.init(millis());

    // Set up serial debugging
    Serial.begin(115200);

    // Initialize state estimator
    _madgwick.initialize();

}

void loop() 
{
    const auto msec = millis();

    vehicleState_t mstate = {};
    getMadgwickState(getDt(), mstate);

    vehicleState_t estate = {};
    getEkfState(msec, estate);

    dump(msec, mstate, estate);

    runLoopDelay(micros());
}
