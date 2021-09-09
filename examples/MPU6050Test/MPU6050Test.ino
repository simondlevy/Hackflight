#include <Wire.h>

#include "HF_full.hpp"
#include "hf_sensors/mpu6050.hpp"

#include <RFT_filters.hpp>

static MPU6050 mpu = MPU6050(MPU6050::AFS_2G, MPU6050::GFS_250DPS);

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion

// parameters for 6 DoF sensor fusion calculations
static constexpr float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
static constexpr float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static constexpr float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // beta parameter for Madgwick quaternion filter
static constexpr float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // zeta parameter for Madgwick quaternion filter

static rft::MadgwickQuaternionFilter6DOF madgwick = rft::MadgwickQuaternionFilter6DOF(beta, zeta);

void setup()
{
    Wire.begin();

    Serial.begin(115200);

    mpu.begin();
}

void loop()
{
    static float ax, ay, az, gx, gy, gz;

    // If data ready bit set, all data registers have new data
    if (mpu.dataReady()) {

        mpu.readData(ax, ay, az, gy, gy, gz);

    }

    static uint32_t lastUpdate;
    uint32_t Now = micros();
    float deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    //    if(lastUpdate - firstUpdate > 10000000uL) {
    //      beta = 0.041; // decrease filter gain after stabilized
    //      zeta = 0.015; // increase gyro bias drift gain after stabilized
    //    }

    // Pass gyro rate as rad/s
    // MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, deltat, beta, zeta, q);
    madgwick.update(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, deltat);

    // Serial print and/or display at 0.5 s rate independent of data rates
    static uint32_t count;
    uint32_t delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

        float q0 = madgwick.q1;
        float q1 = madgwick.q2;
        float q2 = madgwick.q3;
        float q3 = madgwick.q4;

        float yaw   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
        float pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
        float roll  = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        roll  *= 180.0f / PI;

        Serial.print("\nYaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);
        count = millis();
    }
}
