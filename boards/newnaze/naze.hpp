// Board implementation for Naze32 ======================================================
#include <cstdio>
#include <cstdint>

#include <time.h>

#include "board.hpp"
#include "config.hpp"
#include "common.hpp"

#include <SpektrumDSM.h>
#include <MPU6050.h>
#include <Motor.h>

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
        rx = new SpektrumDSM2048();

        // Connect to motors
        motors[0].attach(8);
        motors[1].attach(11);
        motors[2].attach(6);
        motors[3].attach(7);

    }

    virtual void checkReboot(bool pendReboot) override
    {
        if (pendReboot)
            reset(); // noreturn
    }    

    virtual void reboot(void) override
    {
        resetToBootloader();
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
    }


    virtual uint64_t getMicros() override
    {
        return micros();
    }

    virtual bool rcUseSerial(void) override
    {
        rx->begin();
        return true;
    }

    virtual bool rcSerialReady(void) override
    { 
        return rx->frameComplete();
    }

    virtual uint16_t rcReadSerial(uint8_t chan)  
    { 
        uint8_t chanmap[5] = {1, 2, 3, 0, 5};
        return rx->readRawRC(chanmap[chan]);
    }

    virtual uint16_t readPWM(uint8_t chan) override
    {
        (void)chan;
        return 0; // because we're using Spektrum serial RX
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

    virtual uint8_t serialAvailableBytes(void) override
    {
        return Serial.available();
    }

    virtual uint8_t serialReadByte(void) override
    {
        return Serial.read();
    }

    virtual void serialWriteByte(uint8_t c) override
    {
        Serial.write(c);
    }

    virtual void writeMotor(uint8_t index, uint16_t value) override
    {
        //Serial.printf("m%d:%d%s", index+1, value, index==3 ? "\n" : "  ");
        motors[index].setSpeed(value);
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
    SpektrumDSM2048 * rx;

    // Motor support
    BrushlessMotor motors[4];

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
