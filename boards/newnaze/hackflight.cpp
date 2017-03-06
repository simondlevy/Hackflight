#include <Arduino.h>

#include <MPU6050.h>

//#include <hackflight.hpp>
//#include "naze.hpp"

//hf::Hackflight h;

MPU6050 * imu;

void setup(void)
{
    Serial.begin(115200);
    //h.init(new hf::Naze());

    HardwareWire::init(I2CDEV_2);

    delay(100);

    imu = new MPU6050();
    imu->begin(AFS_8G, GFS_2000DPS);

}

void loop(void)
{
    //h.update();

    int16_t gyroAdc[3], accelAdc[3];

    imu->getMotion6Counts(
            &accelAdc[0], &accelAdc[1], &accelAdc[2], 
            &gyroAdc[0], &gyroAdc[1], &gyroAdc[2]);

    Serial.printf("%5d %5d %5d\n", accelAdc[0], accelAdc[1], accelAdc[2]);
}
