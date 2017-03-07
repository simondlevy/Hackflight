// Board implementation for Naze32 ======================================================
#include <cstdio>
#include <cstdint>

#include <time.h>

#include "board.hpp"
#include "config.hpp"
#include "common.hpp"

#include <SpektrumDSM.h>
#include <MPU6050.h>

namespace hf {

class Naze : public Board {

public:
    virtual void init() override
    {
        // Init LEDs
        pinMode(3, OUTPUT);
        pinMode(4, OUTPUT);

        // Configure 
        config.imu.imuLoopMicro = 3500;
        config.rc.rcLoopMilli = 21; 
        config.imu.calibratingGyroMilli = 3500; 
        config.imu.accelCalibrationPeriodMilli = 1400; 
        config.imu.altitudeUpdatePeriodMilli = 25; 
        config.ledFlashCountOnStartup = 20;
        config.imu.acc1G = 4096;// Accel scale 8g (4096 LSB/g)
        config.imu.gyroScale = 16.4f;// 16.4 dps/lsb scalefactor for all Invensense devices

        // Start I^2C
        //;/HardwareWire::init(I2CDEV_2);
        Wire.begin(2);

        // Hang a sec
        delay(100);

        // Start IMU
        imu = new MPU6050();
        imu->begin(AFS_8G, GFS_2000DPS);

        // Start receiver
        //rx = new SpektrumDSM();
    }

    virtual const Config& getConfig() override
    {
        return config;
    }

    virtual void imuRead(int16_t gyroAdc[3], int16_t accelAdc[3]) override
    {
        imu->getMotion6Counts(
                &accelAdc[0], &accelAdc[1], &accelAdc[2], 
                &gyroAdc[0], &gyroAdc[1], &gyroAdc[2]);

        //Serial.printf("%5d %5d %5d\n", accelAdc[0], accelAdc[1], accelAdc[2]);
    }


    virtual uint64_t getMicros() override
    {
        return micros();
    }

    virtual bool rcUseSerial(void) override
    {
        return false;
    }

    virtual uint16_t readPWM(uint8_t chan) override
    {
        (void)chan;
        return 1500;
    }

    virtual void setLed(uint8_t id, bool is_on, float max_brightness) override
    {
        (void)max_brightness;

        digitalWrite(id ? 4 : 3, is_on ? HIGH : LOW);
    }

    virtual void dump(char * msg) override
    {
        Serial.printf("%s", msg);
    }


    virtual void writeMotor(uint8_t index, uint16_t value) override
    {
        (void)index;
        (void)value;
    }

    virtual void showArmedStatus(bool armed) override
    {
        (void)armed;
    }

    virtual void showAuxStatus(uint8_t status) override
    {
        (void)status;
    }
    
    virtual void delayMilliseconds(uint32_t msec) override
    {
        delay(msec);
    }


private:

    // IMU support
    MPU6050 * imu;

    // RC support
    SpektrumDSM * rx;

    // Launch support
    bool ready;

    // needed for spring-mounted throttle stick
    float throttleDemand;
    const float SPRINGY_THROTTLE_INC = .01f;

    // IMU support
    float accel[3];
    float gyro[3];

    // Barometer support
    int baroPressure;

    // Motor support
    float thrusts[4];

    // 100 Hz timestep, used for simulating microsend timer
    float timestep;

    int particleCount;

    // Handles from scene
    int motorList[4];
    int motorJointList[4];
    int quadcopterHandle;
    int accelHandle;

    // Support for reporting status of aux switch (alt-hold, etc.)
    uint8_t auxStatus;
    // Stick demands from controller
    float demands[5];

    // Controller type
    // We currently support these controllers
    enum controller_t { KEYBOARD, DSM, TARANIS, SPEKTRUM, EXTREME3D, PS3 , XBOX360 };
    controller_t controller;

    Config config;
};

} //namespace
