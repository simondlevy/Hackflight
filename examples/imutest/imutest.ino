#include <LSM6DSOSensor.h>
#include <MPU6050.h>

#include <hackflight.h>
#include <firmware/debugging.hpp>
#include <firmware/profiling.hpp>

//static LSM6DSOSensor _lsm6dso(&Wire);

static MPU6050 _mpu6050;

/*
static bool bad(const LSM6DSOStatusTypeDef status)
{
    return status != LSM6DSO_OK;
}*/


void setup()
{
    Serial.begin(0);

    Wire.begin();

    Wire.setClock(1000000); 

    _mpu6050.initialize();

    /*
    if (
            bad(_lsm6dso.begin()) ||
            bad(_lsm6dso.Enable_G())  ||
            bad(_lsm6dso.Enable_X()) ||
            bad(_lsm6dso.Set_X_FS(16)) ||
            bad(_lsm6dso.Set_G_FS(2000))) {*/

    if (!_mpu6050.testConnection()) {
        hf::Debugger::reportForever("Initialization unsuccessful\n");
    }

    _mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    _mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

     _mpu6050.setIntDataReadyEnabled(true);
}

void loop()
{
    // hf::Profiler::report();

    static uint32_t _count;

    //uint8_t status = 0;
    //_lsm6dso.Get_G_DRDY_Status(&status);

    bool status = _mpu6050.getIntDataReadyStatus();

    if (status) {

        int16_t g[3] = {};
        int16_t a[3] = {};
        /*
        _lsm6dso.Get_G_AxesRaw(g);
        _lsm6dso.Get_X_AxesRaw(a);*/

       _mpu6050.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);

        //printf("gx=%d gy=%d gz=%d ax=%d ay=%d az=%d\n",
        //        g[0], g[1], g[2], a[0], a[1], a[2]);

       _count = 0;
    }

    else {
        printf("%d\n", _count++);
    }
}
