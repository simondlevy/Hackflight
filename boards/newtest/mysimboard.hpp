// Board implementation ======================================================
#include <cstdio>
#include <cstdint>

#include <time.h>

#include "board.hpp"
#include "config.hpp"
#include "common.hpp"


namespace hf {

class SimBoard : public Board {
public:
    virtual void init() override
    {
        config.imu.imuLoopMicro = 10000;
        config.imu.calibratingGyroMilli = 100; //long enough to see but not to annoy
    }

    virtual const Config& getConfig() override
    {
        return config;
    }

    virtual void imuRead(int16_t gyroAdc[3], int16_t accelAdc[3]) override
    {
        // Convert from radians to tenths of a degree

        for (int k=0; k<3; ++k) {
            accelAdc[k]  = (int16_t)(400000 * accel[k]);
        }

        gyroAdc[1] = -(int16_t)(1000 * gyro[0]);
        gyroAdc[0] = -(int16_t)(1000 * gyro[1]);
        gyroAdc[2] = -(int16_t)(1000 * gyro[2]);
    }


    virtual uint64_t getMicros() override
    {
        return clock() / 1000;
    }

    virtual bool rcUseSerial(void) override
    {
        return false;
    }

    virtual uint16_t readPWM(uint8_t chan) override
    {
        // Special handling for throttle
        float demand = (chan == 3) ? throttleDemand : demands[chan];

        // Special handling for pitch, roll on PS3, XBOX360
        if (chan < 2) {
           if (controller == PS3)
            demand /= 2;
           if (controller == XBOX360)
            demand /= 1.5;
        }

        // Joystick demands are in [-1,+1]
        int pwm =  (int)(CONFIG_PWM_MIN + (demand + 1) / 2 * (CONFIG_PWM_MAX - CONFIG_PWM_MIN));

        return pwm;
    }

    virtual void dump(char * msg) override
    {
        printf("%s\n", msg);
    }


    virtual void writeMotor(uint8_t index, uint16_t value) override
    {
        thrusts[index] = (value - 1000) / 1000.0f;
    }

    virtual void showArmedStatus(bool armed) override
    {
        //TODO: provide implementtaion
        //if (armed) 
        //    startToast("                    ARMED", 1, 0, 0);
    }

    virtual void showAuxStatus(uint8_t status) override
    {
        if (status != auxStatus) {
            char message[100];
            switch (status) {
                case 1:
                    sprintf(message, "ENTERING ALT-HOLD");
                    break;
                case 2:
                    sprintf(message, "ENTERING GUIDED MODE");
                    break;
                default:
                    sprintf(message, "ENTERING NORMAL MODE");
            }
            //TODO: provide implementtaion
            //startToast(message, 1,1,0);
        }

        auxStatus = status;
    }
    
    virtual void delayMilliseconds(uint32_t msec) override
    {
        if (msec <= 0)
            return;

        uint32_t msec_start = clock() / 1000;
        while (true) {
            uint32_t msec_curr = clock() / 1000;
            if ((msec_curr-msec_start)>msec)
                break;
        }
    }


private:

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
