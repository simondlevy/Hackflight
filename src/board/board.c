#include <stdbool.h>
#include <stdint.h>

#include "revision.h"
#include "../mockduino/drv_i2c.h"
#include "../mockduino/drv_spi.h"
#include "../mockduino/drv_adc.h"

// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;
extern void adcInit(bool);

int hw_revision = 0;

void initBoardSpecific(void)
{

    // determine hardware revision based on clock frequency
    if (hse_value == 8000000)
        hw_revision = NAZE32;
    else if (hse_value == 12000000)
        hw_revision = NAZE32_REV5;

    if (spiInit() == SPI_DEVICE_MPU && hw_revision == NAZE32_REV5)
        hw_revision = NAZE32_SP;

    if (hw_revision != NAZE32_SP)
        i2cInit(I2CDEV_2);

    adcInit(hw_revision >= NAZE32_REV5);
}

