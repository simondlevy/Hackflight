#include <Wire.h>
#include "MPU6050.h"

#include "Madgwick.hpp"

static MPU6050lib mpu;

static float aRes, gRes; // scale resolutions per LSB for the sensors
static int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
static float ax, ay, az;       // Stores the real accel value in g's
static int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
static float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
static float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
static int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
static float temperature;
static float SelfTest[6];
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion
static uint32_t delt_t = 0; // used to control display output rate
static uint32_t count = 0;  // used to control display output rate
static float pitch, yaw, roll;

// parameters for 6 DoF sensor fusion calculations
static float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
static float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
static float deltat = 0.0f;                              // integration interval for both filter schemes
static uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
static uint32_t Now = 0;                                 // used to calculate integration interval

void setup()
{
    Wire.begin();

    Serial.begin(115200);

    mpu.initMPU6050();

    mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
}

void loop()
{
    // If data ready bit set, all data registers have new data
    if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
        mpu.readAccelData(accelCount);  // Read the x/y/z adc values
        aRes = mpu.getAres();

        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes;
        az = (float)accelCount[2] * aRes;

        mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
        gRes = mpu.getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1] * gRes;
        gz = (float)gyroCount[2] * gRes;

        tempCount = mpu.readTempData();  // Read the x/y/z adc values
        temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
    }

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    //    if(lastUpdate - firstUpdate > 10000000uL) {
    //      beta = 0.041; // decrease filter gain after stabilized
    //      zeta = 0.015; // increase gyro bias drift gain after stabilized
    //    }
    // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, deltat, beta, zeta, q);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

        pitch *= 180.0f / PI;
        yaw   *= 180.0f / PI;
        roll  *= 180.0f / PI;

        Serial.print("\nYaw, Pitch, Roll: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);

        Serial.println(" x\t  y\t  z  ");

        Serial.print((int)(1000 * ax)); Serial.print('\t');
        Serial.print((int)(1000 * ay)); Serial.print('\t');
        Serial.print((int)(1000 * az));
        Serial.println(" mg");

        Serial.print((int)(gx)); Serial.print('\t');
        Serial.print((int)(gy)); Serial.print('\t');
        Serial.print((int)(gz));
        Serial.println(" o/s");

        Serial.print((int)(yaw)); Serial.print('\t');
        Serial.print((int)(pitch)); Serial.print('\t');
        Serial.print((int)(roll));
        Serial.println(" ypr");

        Serial.print("rt: "); Serial.print(1.0f / deltat, 2); Serial.println(" Hz");

        count = millis();
    }
}
