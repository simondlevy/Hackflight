/**
 *
 * Copyright (C) 2011-2022 Bitcraze AB, 2025 Simon D. Levy
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
 */

#include <hal/digital.h>
#include <hal/exti.h>
#include <hal/i2cdev.h>
#include <hal/nvic.h>
#include <hal/spi2.h>
#include <hal/time.h>
#include <hal/usb.h>

#include <tasks/imu.hpp>

#include <bosch/bmi088.h>
#include <bosch/bstdr_types.h>

static bstdr_ret_t spi_burst_read(uint8_t dev_id, uint8_t reg_addr,
        uint8_t *reg_data, uint16_t len)
{
    auto accel = dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY;

    digitalWrite(accel ? 19 : 18, LOW);

    spi2_dma_read(reg_addr, reg_data, len);

    digitalWrite(accel ? 19 : 18, HIGH);

    return BSTDR_OK;
}

static bstdr_ret_t spi_burst_write(uint8_t dev_id, uint8_t reg_addr,
        uint8_t *reg_data, uint16_t len)
{
    auto accel = dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY;

    digitalWrite(accel ? 19 : 18, LOW);

    // spi2_dma_transaction(reg_addr, reg_data, len);

    spi2_send_byte(reg_addr);

    for (int i = 0; i < len; i++) {
        spi2_send_byte(reg_data[i]);
    }

    digitalWrite(accel ? 19 : 18, HIGH);

    return BSTDR_OK;
}

static void sensorsBmi088_SPI_deviceInit(struct bmi088_dev *device)
{
    pinMode(18, OUTPUT);
    pinMode(19, OUTPUT);

    digitalWrite(19, HIGH);

    spi2_begin();

    device->gyro_id = BMI088_GYRO_I2C_ADDR_PRIMARY;
    device->interface = BMI088_SPI_INTF;
    device->read = (bmi088_com_fptr_t)spi_burst_read;
    device->write = (bmi088_com_fptr_t)spi_burst_write;
}

static struct bmi088_dev bmi088Dev;

//////////////////////////////////////////////////////////////////////////////

bool ImuTask::device_init(void)
{
    bmi088Dev.accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY;
    bmi088Dev.delay_ms = delay;

    void sensorsBmi088_SPI_deviceInit(struct bmi088_dev *device);
    sensorsBmi088_SPI_deviceInit(&bmi088Dev);

    auto rslt = bmi088_gyro_init(&bmi088Dev); // initialize the device

    bool success = true;

    if (rslt == BSTDR_OK) {

        struct bmi088_int_cfg intConfig;

        bmi088Dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
        rslt |= bmi088_set_gyro_power_mode(&bmi088Dev);

        bmi088Dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
        bmi088Dev.gyro_cfg.range = BMI088_GYRO_RANGE_2000_DPS;
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

        success = false;
    }

    rslt |= bmi088_accel_switch_control(&bmi088Dev, BMI088_ACCEL_POWER_ENABLE);

    delay(5);

    rslt = bmi088_accel_init(&bmi088Dev);

    if (rslt == BSTDR_OK) {

        bmi088Dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;
        rslt |= bmi088_set_accel_power_mode(&bmi088Dev);
        delay(10);

        bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
        bmi088Dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
        rslt |= bmi088_set_accel_meas_conf(&bmi088Dev);

        struct bmi088_sensor_data acc;
        rslt |= bmi088_get_accel_data(&acc, &bmi088Dev);
    }
    else {
        success = false;
    }

    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

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

    return success;
}

void ImuTask::device_readRaw(
        int16_t & gx, int16_t & gy, int16_t & gz, 
        int16_t & ax, int16_t & ay, int16_t & az)
{
    struct bmi088_sensor_data dataOut = {};

    bmi088_get_gyro_data(&dataOut, &bmi088Dev);

    gx = dataOut.x;
    gy = dataOut.y;
    gz = dataOut.z;

    bmi088_get_accel_data(&dataOut, &bmi088Dev);

    ax = dataOut.x;
    ay = dataOut.y;
    az = dataOut.z;
}
