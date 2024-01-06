#include <string.h>

#include <crossplatform.h>

#include <arduino/digital.h>

#include <hal/spi2.h>

#include <bstdr_types.h>
#include <bmi088.h>
#include <nvicconf.h>
#include <pinmap.h>

static bstdr_ret_t spi_burst_read(uint8_t dev_id, uint8_t reg_addr,
        uint8_t *reg_data, uint16_t len)
{
    auto accel = dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY;

    digitalWrite(accel ? PIN_BOLT_ACCEL_CS : PIN_BOLT_GYRO_CS, LOW);

    spi2_dma_read(reg_addr, reg_data, len);

    digitalWrite(accel ? PIN_BOLT_ACCEL_CS : PIN_BOLT_GYRO_CS, HIGH);

    return BSTDR_OK;
}

static bstdr_ret_t spi_burst_write(uint8_t dev_id, uint8_t reg_addr,
        uint8_t *reg_data, uint16_t len)
{
    auto accel = dev_id == BMI088_ACCEL_I2C_ADDR_PRIMARY;

    digitalWrite(accel ? PIN_BOLT_ACCEL_CS : PIN_BOLT_GYRO_CS, LOW);

    // spi2_dma_transaction(reg_addr, reg_data, len);

    spi2_send_byte(reg_addr);

    for (int i = 0; i < len; i++) {
        spi2_send_byte(reg_data[i]);
    }

    digitalWrite(accel ? PIN_BOLT_ACCEL_CS : PIN_BOLT_GYRO_CS, HIGH);

    return BSTDR_OK;
}

void sensorsBmi088_SPI_deviceInit(struct bmi088_dev *device)
{
    pinMode(PIN_BOLT_GYRO_CS, OUTPUT);
    pinMode(PIN_BOLT_ACCEL_CS, OUTPUT);

    digitalWrite(PIN_BOLT_ACCEL_CS, HIGH);

    spi2_begin();

    device->gyro_id = BMI088_GYRO_I2C_ADDR_PRIMARY;
    device->interface = BMI088_SPI_INTF;
    device->read = (bmi088_com_fptr_t)spi_burst_read;
    device->write = (bmi088_com_fptr_t)spi_burst_write;
}


