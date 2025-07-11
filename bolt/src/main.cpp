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

#define __main
#include <Arduino.h>

#include <stm32fxxx.h>

#include <hal/digital.h>
#include <hal/exti.h>
#include <hal/i2cdev.h>
#include <hal/nvic.h>
#include <hal/spi2.h>
#include <hal/uart.h>
#include <hal/time.h>
#include <hal/usb.h>

#include <bosch/bmi088.h>
#include <bosch/bstdr_types.h>

#include <system.h>

#include <tasks/imu.hpp>
#include <tasks/zranger.hpp>

#include <vl53l1.hpp>

#include <motors.h>

#include <st/vl53l1_api.h>

static const uint8_t LED_PIN = 4;
static const uint8_t FLOWDECK_CS_PIN = 11;

static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;

static VL53L1 _vl53l1;

void debug(const char * msg)
{
}

void error(const char * msg)
{
}

void assertFail(char *exp, char *file, int line)
{
    portDISABLE_INTERRUPTS();
    debug("SYSTEM: Assert failed");

    motorsStop();

    NVIC_SystemReset();
}

// ZRangerTask ---------------------------------------------------------------

void ZRangerTask::hardware_init()
{
    static const uint8_t VL53L1_DEFAULT_ADDRESS = 0x29;

    _vl53l1.init(&deckBus, VL53L1_DEFAULT_ADDRESS);

    if (_vl53l1.begin()) {
        debug("ZRANGER: Z-down sensor [OK]");
    }
    else {
        debug("ZRANGER: Z-down sensor [FAIL]");
    }

    _vl53l1.setDistanceMode(VL53L1::DISTANCE_MODE_MEDIUM);
    _vl53l1.setTimingBudgetMsec(25);
}

float ZRangerTask::hardware_read()
{
    return _vl53l1.readDistance();
}

// ---------------------------------------------------------------------------

int main() 
{
    nvicInit();
    extiInit();
    usecTimerInit();
    i2cdevInit();
    usbInit();
    uartInit(115200); // RaspberryPi comms
    SPI.begin();
    systemInit(LED_PIN, FLOWDECK_CS_PIN);

    while(true) {
    }

    return 0;
}

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

static ImuTask * _imuTask;

extern "C" {

    void __attribute__((used)) EXTI14_Callback(void) 
    {
        _imuTask->dataAvailableCallback();
    }
}

static struct bmi088_dev bmi088Dev;

void ImuTask::readGyroRaw(Axis3i16* dataOut)
{
    bmi088_get_gyro_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

void ImuTask::readAccelRaw(Axis3i16 * dataOut)
{
    bmi088_get_accel_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

const bool systemIsLedInverted() 
{
    return true;
}

bool systemUartReadByte(uint8_t * byte)
{
    return uartGetData(1, byte);
}

void systemUartWriteByte(const uint8_t byte)
{
    uartSendDataDmaBlocking(1, (uint8_t *)&byte);
}
 
void ImuTask::deviceInit(void)
{
    _imuTask = this;

    bmi088Dev.accel_id = BMI088_ACCEL_I2C_ADDR_PRIMARY;
    bmi088Dev.delay_ms = delay;

    void sensorsBmi088_SPI_deviceInit(struct bmi088_dev *device);
    sensorsBmi088_SPI_deviceInit(&bmi088Dev);

    auto rslt = bmi088_gyro_init(&bmi088Dev); // initialize the device

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
}

extern "C" {

    VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t usec)
    {
        void delayMicroseconds(const uint32_t usec);

        delayMicroseconds(usec);

        return VL53L1_ERROR_NONE;
    }

    VL53L1_Error VL53L1_WriteMulti(VL53L1_Dev_t *pdev, uint16_t index, 
            uint8_t * pdata, uint32_t count)
    {
        return i2cdevWriteReg16(
                (I2C_Dev*)pdev->I2Cx, pdev->devAddr, index, count, pdata) ?
            VL53L1_ERROR_NONE : 
            VL53L1_ERROR_CONTROL_INTERFACE;
    }

    VL53L1_Error VL53L1_ReadMulti(VL53L1_Dev_t *pdev, uint16_t index, 
            uint8_t * pdata, uint32_t   count)
    {
        return i2cdevReadReg16(
                (I2C_Dev*)pdev->I2Cx, pdev->devAddr, index, count, pdata) ?
            VL53L1_ERROR_NONE : 
            VL53L1_ERROR_CONTROL_INTERFACE;
    }


}

