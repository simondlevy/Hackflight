/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <math.h>

#include <autoconf.h>

#include <hal/i2cdev.h>

#include <tasks/core.hpp>

#include <bstdr_types.h>
#include <bmi088.h>
#include <crossplatform.h>
#include <console.h>
#include <configblock.hpp>

static ImuTask * _imuTask;

extern "C" {

#include "bmp3.h"

    void __attribute__((used)) EXTI14_Callback(void) 
    {
        _imuTask->dataAvailableCallback();
    }
}

static const uint8_t SENSORS_BMI088_ACCEL_FS_CFG = BMI088_ACCEL_RANGE_24G;
static const uint8_t SENSORS_BMI088_GYRO_FS_CFG = BMI088_GYRO_RANGE_2000_DPS;

static struct bmi088_dev bmi088Dev;

bstdr_ret_t i2c_burst_read(
        uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    return i2cdevReadReg8(I2C3_DEV, dev_id, reg_addr, (uint16_t) len, reg_data) ?
        BSTDR_OK :
        BSTDR_E_CON_ERROR;
}

bstdr_ret_t i2c_burst_write(
        uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    return i2cdevWriteReg8(I2C3_DEV, dev_id,reg_addr,(uint16_t) len, reg_data) ?
        BSTDR_OK :
        BSTDR_E_CON_ERROR;
}

bool ImuTask::gyroSelfTest()
{
    auto testStatus = true;
    
    auto readResult = bmi088_get_gyro_data(
            (struct bmi088_sensor_data*)&gyroRaw, 
            &bmi088Dev);

    if ((readResult != BMI088_OK) || 
            (gyroRaw.x == 0 && gyroRaw.y == 0 && gyroRaw.z == 0)) {
        consolePrintf("IMU: Gyro returning x=0 y=0 z=0 [FAILED]\n");
        testStatus = false;
    }

    int8_t gyroResult = 0;

    bmi088_perform_gyro_selftest(&gyroResult, &bmi088Dev);

    if (gyroResult == BMI088_SELFTEST_PASS) {
        consolePrintf("IMU: Gyro self-test [OK]\n");
    }
    else {
        consolePrintf("IMU: Gyro self-test [FAILED]\n");
        testStatus = false;
    }

    return testStatus;
}


void ImuTask::readGyro(Axis3i16* dataOut)
{
    bmi088_get_gyro_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

void ImuTask::readAccel(Axis3i16* dataOut)
{
    bmi088_get_accel_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}


void ImuTask::deviceInit(void)
{
    _imuTask = this;

    bmi088Dev.accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY;
    bmi088Dev.delay_ms = delay;

#ifdef CONFIG_SENSORS_BMI088_SPI
    void sensorsBmi088_SPI_deviceInit(struct bmi088_dev *device);
    consolePrintf("IMU: BMI088: Using SPI interface.\n");
    sensorsBmi088_SPI_deviceInit(&bmi088Dev);
#else
    void sensorsBmi088_I2C_deviceInit(struct bmi088_dev *device);
    consolePrintf("IMU: BMI088: Using I2C interface.\n");
    sensorsBmi088_I2C_deviceInit(&bmi088Dev);
#endif

    auto rslt = bmi088_gyro_init(&bmi088Dev); // initialize the device

    if (rslt == BSTDR_OK) {

        struct bmi088_int_cfg intConfig;

        consolePrintf("IMU: BMI088 Gyro connection [OK].\n");
        
        bmi088Dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
        rslt |= bmi088_set_gyro_power_mode(&bmi088Dev);
        
        bmi088Dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
        bmi088Dev.gyro_cfg.range = SENSORS_BMI088_GYRO_FS_CFG;
        bmi088Dev.gyro_cfg.odr = BMI088_GYRO_BW_116_ODR_1000_HZ;
        rslt |= bmi088_set_gyro_meas_conf(&bmi088Dev);

        intConfig.gyro_int_channel = BMI088_INT_CHANNEL_3;
        intConfig.gyro_int_type = BMI088_GYRO_DATA_RDY_INT;
        intConfig.gyro_int_pin_3_cfg.enable_int_pin = 1;
        intConfig.gyro_int_pin_3_cfg.lvl = 1;
        intConfig.gyro_int_pin_3_cfg.output_mode = 0;
        
        rslt = bmi088_set_gyro_int_config(&intConfig, &bmi088Dev);

        delay(50);
        struct bmi088_sensor_data gyr;
        rslt |= bmi088_get_gyro_data(&gyr, &bmi088Dev);
    }

    else {
        consolePrintf("IMU: BMI088 Gyro connection [FAIL]\n");
        didInit = false;
    }

    rslt |= bmi088_accel_switch_control(&bmi088Dev, BMI088_ACCEL_POWER_ENABLE);

    delay(5);

    rslt = bmi088_accel_init(&bmi088Dev);

    if (rslt == BSTDR_OK) {

        consolePrintf("IMU: BMI088 Accel connection [OK]\n");
        
        bmi088Dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;
        rslt |= bmi088_set_accel_power_mode(&bmi088Dev);
        delay(10);

        bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
        bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
        bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
        rslt |= bmi088_set_accel_meas_conf(&bmi088Dev);

        struct bmi088_sensor_data acc;
        rslt |= bmi088_get_accel_data(&acc, &bmi088Dev);
    }

    else {
        consolePrintf("IMU: BMI088 Accel connection [FAIL]\n");
        didInit = false;
    }

    didInit = true;

    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
    dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

    // Enable the interrupt on PC14
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);

    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    portDISABLE_INTERRUPTS();
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearITPendingBit(EXTI_Line14);
    portENABLE_INTERRUPTS();
}
