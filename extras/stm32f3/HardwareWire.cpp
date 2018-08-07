extern "C" {

#include "Arduino.h"
#include "Wire.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "gpio.h"
#include "system.h"

#define I2C_SHORT_TIMEOUT            ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * I2C_SHORT_TIMEOUT))
#define I2C_DEFAULT_TIMEOUT          I2C_LONG_TIMEOUT

#define I2C1_SCL_GPIO        GPIOB
#define I2C1_SCL_GPIO_AF     GPIO_AF_4
#define I2C1_SCL_PIN         GPIO_Pin_6
#define I2C1_SCL_PIN_SOURCE  GPIO_PinSource6
#define I2C1_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_GPIO        GPIOB
#define I2C1_SDA_GPIO_AF     GPIO_AF_4
#define I2C1_SDA_PIN         GPIO_Pin_7
#define I2C1_SDA_PIN_SOURCE  GPIO_PinSource7
#define I2C1_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOB

#if !defined(I2C2_SCL_GPIO)
#define I2C2_SCL_GPIO        GPIOF
#define I2C2_SCL_GPIO_AF     GPIO_AF_4
#define I2C2_SCL_PIN         GPIO_Pin_6
#define I2C2_SCL_PIN_SOURCE  GPIO_PinSource6
#define I2C2_SCL_CLK_SOURCE  RCC_AHBPeriph_GPIOF
#define I2C2_SDA_GPIO        GPIOA
#define I2C2_SDA_GPIO_AF     GPIO_AF_4
#define I2C2_SDA_PIN         GPIO_Pin_10
#define I2C2_SDA_PIN_SOURCE  GPIO_PinSource10
#define I2C2_SDA_CLK_SOURCE  RCC_AHBPeriph_GPIOA

#endif

static uint32_t i2cTimeout;

static volatile uint16_t i2c1ErrorCount = 0;
static volatile uint16_t i2c2ErrorCount = 0;

static I2C_TypeDef *I2Cx = NULL;

///////////////////////////////////////////////////////////////////////////////
// I2C TimeoutUserCallback
///////////////////////////////////////////////////////////////////////////////

static bool i2cOverClock;

void i2cSetOverclock(uint8_t OverClock)
{
    i2cOverClock = (OverClock) ? true : false;
}

void i2cInitPort(I2C_TypeDef *I2Cx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    if (I2Cx == I2C1) {
        RCC_AHBPeriphClockCmd(I2C1_SCL_CLK_SOURCE | I2C1_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

        //i2cUnstick(I2Cx);                                         // Clock out stuff to make sure slaves arent stuck

        GPIO_PinAFConfig(I2C1_SCL_GPIO, I2C1_SCL_PIN_SOURCE, I2C1_SCL_GPIO_AF);
        GPIO_PinAFConfig(I2C1_SDA_GPIO, I2C1_SDA_PIN_SOURCE, I2C1_SDA_GPIO_AF);

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&I2C_InitStructure);

        // Init pins

        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_InitStructure.GPIO_Pin = I2C1_SCL_PIN;
        GPIO_Init(I2C1_SCL_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = I2C1_SDA_PIN;
        GPIO_Init(I2C1_SDA_GPIO, &GPIO_InitStructure);

        I2C_StructInit(&I2C_InitStructure);

        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        I2C_InitStructure.I2C_DigitalFilter = 0x00;
        I2C_InitStructure.I2C_OwnAddress1 = 0x00;
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
        if (i2cOverClock) {
            I2C_InitStructure.I2C_Timing = 0x00500E30; // 1000 Khz, 72Mhz Clock, Analog Filter Delay ON, Setup 40, Hold 4.
        } else {
            I2C_InitStructure.I2C_Timing = 0x00E0257A; // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10
        }
        //I2C_InitStructure.I2C_Timing              = 0x8000050B;


        I2C_Init(I2C1, &I2C_InitStructure);

        I2C_Cmd(I2C1, ENABLE);
    }

    if (I2Cx == I2C2) {
        RCC_AHBPeriphClockCmd(I2C2_SCL_CLK_SOURCE | I2C2_SDA_CLK_SOURCE, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);

        //i2cUnstick(I2Cx);                                         // Clock out stuff to make sure slaves arent stuck

        GPIO_PinAFConfig(I2C2_SCL_GPIO, I2C2_SCL_PIN_SOURCE, I2C2_SCL_GPIO_AF);
        GPIO_PinAFConfig(I2C2_SDA_GPIO, I2C2_SDA_PIN_SOURCE, I2C2_SDA_GPIO_AF);

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&I2C_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

        GPIO_InitStructure.GPIO_Pin = I2C2_SCL_PIN;
        GPIO_Init(I2C2_SCL_GPIO, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = I2C2_SDA_PIN;
        GPIO_Init(I2C2_SDA_GPIO, &GPIO_InitStructure);

        I2C_StructInit(&I2C_InitStructure);

        I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
        I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
        I2C_InitStructure.I2C_DigitalFilter = 0x00;
        I2C_InitStructure.I2C_OwnAddress1 = 0x00;
        I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
        I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

        // FIXME timing is board specific
        //I2C_InitStructure.I2C_Timing = 0x00310309; // //400kHz I2C @ 8MHz input -> PRESC=0x0, SCLDEL=0x3, SDADEL=0x1, SCLH=0x03, SCLL=0x09 - value from TauLabs/Sparky
        // ^ when using this setting and after a few seconds of a scope probe being attached to the I2C bus it was observed that the bus enters
        // a busy state and does not recover.

        if (i2cOverClock) {
            I2C_InitStructure.I2C_Timing = 0x00500E30; // 1000 Khz, 72Mhz Clock, Analog Filter Delay ON, Setup 40, Hold 4.
        } else {
            I2C_InitStructure.I2C_Timing = 0x00E0257A; // 400 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10
        }

        //I2C_InitStructure.I2C_Timing              = 0x8000050B;

        I2C_Init(I2C2, &I2C_InitStructure);

        I2C_Cmd(I2C2, ENABLE);
    }
}

// ======================================================================================

void HardwareWire::begin(uint8_t bus)
{
    I2Cx = (bus == 2) ? I2C2 : I2C1;
    i2cInitPort(I2Cx);
}

void HardwareWire::beginTransmission(uint8_t address)
{
    this->addr = address << 1;

    this->reg = 0x00;
    this->data = 0x00;
}

uint8_t HardwareWire::write(uint8_t value)
{
    if (this->reg) {
        this->data = value;
    }
    else {
        this->reg = value;
    }

    return 1; // one byte "written"
}

uint8_t HardwareWire::endTransmission(bool stop)
{
    // This is done before a read request
    if (!stop) return 0; // "success"

    // Test on BUSY Flag 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return 1;
        }
    }

    // Configure slave address, nbytes, reload, end mode and start or stop generation 
    I2C_TransferHandling(I2Cx, this->addr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    // Wait until TXIS flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return 2;
        }
    }

    // Send Register address 
    I2C_SendData(I2Cx, (uint8_t) this->reg);

    // Wait until TCR flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
    {
        if ((i2cTimeout--) == 0) {
            return 3;
        }
    }

    // Configure slave address, nbytes, reload, end mode and start or stop generation 
    I2C_TransferHandling(I2Cx, this->addr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    // Wait until TXIS flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return 4;
        }
    }

    // Write data to TXDR 
    I2C_SendData(I2Cx, this->data);

    // Wait until STOPF flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return 5;
        }
    }

    // Clear STOPF flag 
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return 0; // success
}

uint8_t HardwareWire::requestFrom(uint8_t address, uint8_t quantity)
{
    address <<= 1;
    this->avail = 0;
    this->bufpos = 0;

    // Test on BUSY Flag 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return this->avail;
        }
    }

    // Configure slave address, nbytes, reload, end mode and start or stop generation 
    I2C_TransferHandling(I2Cx, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

    // Wait until TXIS flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return this->avail;
        }
    }

    // Send Register address 
    I2C_SendData(I2Cx, (uint8_t) this->reg);

    // Wait until TC flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET) {
        if ((i2cTimeout--) == 0) {
            return this->avail;
        }
    }

    // Configure slave address, nbytes, reload, end mode and start or stop generation 
    I2C_TransferHandling(I2Cx, address, quantity, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    // Wait until all data are received 
    while (quantity) {
        // Wait until RXNE flag is set 
        i2cTimeout = I2C_DEFAULT_TIMEOUT;
        while (I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET) {
            if ((i2cTimeout--) == 0) {
                return this->avail;
            }
        }

        // Read data from RXDR 
        this->buffer[this->avail++] = I2C_ReceiveData(I2Cx);

        // Decrement the read bytes counter 
        quantity--;
    }

    // Wait until STOPF flag is set 
    i2cTimeout = I2C_DEFAULT_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return this->avail;
        }
    }

    // Clear STOPF flag 
    I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

    return this->avail;
}

uint8_t HardwareWire::available(void)
{
    return this->avail;
}

uint8_t HardwareWire::read(void)
{
    this->avail--;

    return this->buffer[this->bufpos++];
}

HardwareWire Wire;

} // extern "C"
