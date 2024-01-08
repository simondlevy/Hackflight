/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * along with this program. If not, see <http:
 *
 *
 * i2cdev.c - Functions to write to I2C devices
 */

#include <string.h>

#include <nvicconf.h>

#include <cfassert.h>

#include <arduino/time.h>

#include "i2cdev.h"


#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000
#define I2C_DEFAULT_DECK_CLOCK_SPEED                400000

#define I2C_NO_BLOCK				    0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2
#define I2C_MESSAGE_TIMEOUT     M2T(1000)

#define I2CDEV_CLK_TS (10)

#define I2CDEV_I2C1_PIN_SDA GPIO_Pin_7
#define I2CDEV_I2C1_PIN_SCL GPIO_Pin_6

#define I2CDEV_NO_MEM_ADDR  0xFF

#define I2C_NO_INTERNAL_ADDRESS   0xFFFF


static void gpioWaitForHigh(GPIO_TypeDef *gpio, uint16_t pin, uint16_t timeout_us)
{
    uint64_t start = micros();
    while (GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && micros() - start <= timeout_us)
    {
    }
}



static const I2cDef sensorBusDef =
{
    .i2cPort            = I2C3,
    .i2cPerif           = RCC_APB1Periph_I2C3,
    .i2cEVIRQn          = I2C3_EV_IRQn,
    .i2cERIRQn          = I2C3_ER_IRQn,
    .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioSCLPerif       = RCC_AHB1Periph_GPIOA,
    .gpioSCLPort        = GPIOA,
    .gpioSCLPin         = GPIO_Pin_8,
    .gpioSCLPinSource   = GPIO_PinSource8,
    .gpioSDAPerif       = RCC_AHB1Periph_GPIOC,
    .gpioSDAPort        = GPIOC,
    .gpioSDAPin         = GPIO_Pin_9,
    .gpioSDAPinSource   = GPIO_PinSource9,
    .gpioAF             = GPIO_AF_I2C3,
    .dmaPerif           = RCC_AHB1Periph_DMA1,
    .dmaChannel         = DMA_Channel_3,
    .dmaRxStream        = DMA1_Stream2,
    .dmaRxIRQ           = DMA1_Stream2_IRQn,
    .dmaRxTCFlag        = DMA_FLAG_TCIF2,
    .dmaRxTEFlag        = DMA_FLAG_TEIF2,
};

I2cDrv sensorsBus =
{
    .def                = &sensorBusDef,
};

static const I2cDef deckBusDef =
{
    .i2cPort            = I2C1,
    .i2cPerif           = RCC_APB1Periph_I2C1,
    .i2cEVIRQn          = I2C1_EV_IRQn,
    .i2cERIRQn          = I2C1_ER_IRQn,
    .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
    .gpioSCLPerif       = RCC_AHB1Periph_GPIOB,
    .gpioSCLPort        = GPIOB,
    .gpioSCLPin         = GPIO_Pin_6,
    .gpioSCLPinSource   = GPIO_PinSource6,
    .gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
    .gpioSDAPort        = GPIOB,
    .gpioSDAPin         = GPIO_Pin_7,
    .gpioSDAPinSource   = GPIO_PinSource7,
    .gpioAF             = GPIO_AF_I2C1,
    .dmaPerif           = RCC_AHB1Periph_DMA1,
    .dmaChannel         = DMA_Channel_1,
    .dmaRxStream        = DMA1_Stream0,
    .dmaRxIRQ           = DMA1_Stream0_IRQn,
    .dmaRxTCFlag        = DMA_FLAG_TCIF0,
    .dmaRxTEFlag        = DMA_FLAG_TEIF0,
};


I2cDrv deckBus =
{
    .def                = &deckBusDef,
};

static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, 
        uint16_t pinSCL, uint16_t pinSDA)
{
    GPIO_SetBits(portSDA, pinSDA);

    while(GPIO_ReadInputDataBit(portSDA, pinSDA) == Bit_RESET)
    {

        GPIO_SetBits(portSCL, pinSCL);
        gpioWaitForHigh(portSCL, pinSCL, 10 * 1000);
        delayMicroseconds(I2CDEV_CLK_TS);
        GPIO_ResetBits(portSCL, pinSCL);
        delayMicroseconds(I2CDEV_CLK_TS);
        GPIO_SetBits(portSCL, pinSCL);
        delayMicroseconds(I2CDEV_CLK_TS);
    }

    GPIO_SetBits(portSCL, pinSCL);
    delayMicroseconds(I2CDEV_CLK_TS);
    GPIO_ResetBits(portSDA, pinSDA);
    delayMicroseconds(I2CDEV_CLK_TS);
    GPIO_ResetBits(portSCL, pinSCL);
    delayMicroseconds(I2CDEV_CLK_TS);

    GPIO_SetBits(portSDA, pinSDA);
    GPIO_SetBits(portSCL, pinSCL);
    gpioWaitForHigh(portSCL, pinSCL, 10 * 1000);
    gpioWaitForHigh(portSDA, pinSDA, 10 * 1000);
}


static void i2cdrvStartTransfer(I2cDrv *i2c)
{
    ASSERT_DMA_SAFE(i2c->txMessage.buffer);

    if (i2c->txMessage.direction == i2cRead)
    {
        i2c->DMAStruct.DMA_BufferSize = i2c->txMessage.messageLength;
        i2c->DMAStruct.DMA_Memory0BaseAddr = (uint32_t)i2c->txMessage.buffer;
        DMA_Init(i2c->def->dmaRxStream, &i2c->DMAStruct);
        DMA_Cmd(i2c->def->dmaRxStream, ENABLE);
    }

    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT, ENABLE);
    i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
}

static void i2cTryNextMessage(I2cDrv* i2c)
{
    i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

static void i2cNotifyClient(I2cDrv* i2c)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2c->isBusFreeSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void i2cdrvDmaSetupBus(I2cDrv* i2c)
{

    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(i2c->def->dmaPerif, ENABLE);


    i2c->DMAStruct.DMA_Channel = i2c->def->dmaChannel;
    i2c->DMAStruct.DMA_PeripheralBaseAddr = (uint32_t)&i2c->def->i2cPort->DR;
    i2c->DMAStruct.DMA_Memory0BaseAddr = 0;
    i2c->DMAStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    i2c->DMAStruct.DMA_BufferSize = 0;
    i2c->DMAStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    i2c->DMAStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    i2c->DMAStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    i2c->DMAStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    i2c->DMAStruct.DMA_Mode = DMA_Mode_Normal;
    i2c->DMAStruct.DMA_Priority = DMA_Priority_High;
    i2c->DMAStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    i2c->DMAStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    i2c->DMAStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    i2c->DMAStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    NVIC_InitStructure.NVIC_IRQChannel = i2c->def->dmaRxIRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{
    I2C_InitTypeDef I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;


    RCC_AHB1PeriphClockCmd(i2c->def->gpioSDAPerif, ENABLE);
    RCC_AHB1PeriphClockCmd(i2c->def->gpioSCLPerif, ENABLE);

    RCC_APB1PeriphClockCmd(i2c->def->i2cPerif, ENABLE);


    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; 
    GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; 
    GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);

    i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, 
            i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);


    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = i2c->def->gpioSCLPin; 
    GPIO_Init(i2c->def->gpioSCLPort, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  i2c->def->gpioSDAPin; 
    GPIO_Init(i2c->def->gpioSDAPort, &GPIO_InitStructure);


    GPIO_PinAFConfig(i2c->def->gpioSCLPort, i2c->def->gpioSCLPinSource, i2c->def->gpioAF);
    GPIO_PinAFConfig(i2c->def->gpioSDAPort, i2c->def->gpioSDAPinSource, i2c->def->gpioAF);


    I2C_DeInit(i2c->def->i2cPort);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = i2c->def->i2cClockSpeed;
    I2C_Init(i2c->def->i2cPort, &I2C_InitStructure);


    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_ERR, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cEVIRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_I2C_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = i2c->def->i2cERIRQn;
    NVIC_Init(&NVIC_InitStructure);

    i2cdrvDmaSetupBus(i2c);
}


static void createMessage(
        I2cMessage *message,
        uint8_t  slaveAddress,
        I2cDirection  direction,
        uint32_t length,
        const uint8_t *buffer)
{
    ASSERT_DMA_SAFE(buffer);
    message->slaveAddress = slaveAddress;
    message->direction = direction;
    message->messageLength = length;
    message->status = i2cAck;
    message->buffer = (uint8_t *)buffer;
    message->nbrOfRetries = I2C_MAX_RETRIES;
}

static void i2cdrvCreateMessage(
        I2cMessage *message,
        uint8_t  slaveAddress,
        I2cDirection  direction,
        uint32_t length,
        const uint8_t *buffer)
{
    createMessage(message, slaveAddress, direction, length, buffer);
    message->isInternal16bit = false;
    message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
}

static void i2cdrvCreateMessage16(
        I2cMessage *message,
        uint8_t  slaveAddress,
        bool IsInternal16,
        uint16_t intAddress,
        I2cDirection  direction,
        uint32_t length,
        const uint8_t  *buffer)
{
    createMessage(message, slaveAddress, direction, length, buffer);
    message->isInternal16bit = IsInternal16;
    message->internalAddress = intAddress;
}

static void i2cdrvClearDMA(I2cDrv* i2c)
{
    DMA_Cmd(i2c->def->dmaRxStream, DISABLE);
    DMA_ClearITPendingBit(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag);
    I2C_DMACmd(i2c->def->i2cPort, DISABLE);
    I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
    DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, DISABLE);
}

static bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message)
{
    bool status = false;

    xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); 

    memcpy((char*)&i2c->txMessage, (char*)message, sizeof(I2cMessage));

    i2cdrvStartTransfer(i2c);

    if (xSemaphoreTake(i2c->isBusFreeSemaphore, I2C_MESSAGE_TIMEOUT) == pdTRUE) {
        if (i2c->txMessage.status == i2cAck) {
            status = true;
        }
    }
    else {
        i2cdrvClearDMA(i2c);
        i2cdrvTryToRestartBus(i2c);

    }
    xSemaphoreGive(i2c->isBusFreeMutex);

    return status;
}

static void i2cdrvEventIsrHandler(I2cDrv* i2c)
{
    uint16_t SR1;
    uint16_t SR2;


    SR1 = i2c->def->i2cPort->SR1;


    if (SR1 & I2C_SR1_SB) {
        i2c->messageIndex = 0;

        if(i2c->txMessage.direction == i2cWrite ||
                i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS) {
            I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1,
                    I2C_Direction_Transmitter);
        }
        else {
            I2C_AcknowledgeConfig(i2c->def->i2cPort, ENABLE);
            I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1,
                    I2C_Direction_Receiver);
        }
    }

    else if (SR1 & I2C_SR1_ADDR) {
        if(i2c->txMessage.direction == i2cWrite ||
                i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS) {
            SR2 = i2c->def->i2cPort->SR2;                              
            if (i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS) {
                if (i2c->txMessage.isInternal16bit) {
                    I2C_SendData(i2c->def->i2cPort, 
                            (i2c->txMessage.internalAddress & 0xFF00) >> 8);
                    I2C_SendData(i2c->def->i2cPort, 
                            (i2c->txMessage.internalAddress & 0x00FF));
                }
                else {
                    I2C_SendData(i2c->def->i2cPort, 
                            (i2c->txMessage.internalAddress & 0x00FF));
                }
                i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
            }
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, ENABLE);       
        }
        else { if(i2c->txMessage.messageLength == 1)
            {
                I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
            }
            else {
                I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); 
            }
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
            DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, ENABLE);
            I2C_DMACmd(i2c->def->i2cPort, ENABLE); 

            DMA_Cmd(i2c->def->dmaRxStream, ENABLE);

            __DMB();                         
            SR2 = i2c->def->i2cPort->SR2;   
        }
    }
    else if (SR1 & I2C_SR1_BTF) {
        SR2 = i2c->def->i2cPort->SR2;
        if (SR2 & I2C_SR2_TRA) {
            if (i2c->txMessage.direction == i2cRead) {
                i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); 
            }
            else {
                i2cNotifyClient(i2c);

                i2cTryNextMessage(i2c);
            }
        }
        else {
            ASSERT(1);
            i2c->txMessage.buffer[i2c->messageIndex++] = 
                I2C_ReceiveData(i2c->def->i2cPort);
            if(i2c->messageIndex == i2c->txMessage.messageLength)
            {
                i2cNotifyClient(i2c);

                i2cTryNextMessage(i2c);
            }
        }


        while (i2c->def->i2cPort->CR1 & 0x0100) { ; }
    }

    else if (SR1 & I2C_SR1_RXNE) {
        i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
        if(i2c->messageIndex == i2c->txMessage.messageLength)
        {
            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);   
        }
    }

    else if (SR1 & I2C_SR1_TXE) {
        if (i2c->txMessage.direction == i2cRead) {

            I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
        }
        else {
            I2C_SendData(i2c->def->i2cPort, i2c->txMessage.buffer[i2c->messageIndex++]);
            if(i2c->messageIndex == i2c->txMessage.messageLength) {

                I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
                __DMB();
            }
        }
    }
}

static void i2cdrvErrorIsrHandler(I2cDrv* i2c)
{
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_AF)) {
        if(i2c->txMessage.nbrOfRetries-- > 0) {

            i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
        }
        else {

            i2c->txMessage.status = i2cNack;
            i2cNotifyClient(i2c);
            i2cTryNextMessage(i2c);
        }
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_AF);
    }
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_BERR)) {
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_BERR);
    }
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_OVR)) {
        I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_OVR);
    }
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_ARLO)) {
        I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_ARLO);
    }
}

static void i2cdrvDmaIsrHandler(I2cDrv* i2c)
{
    if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag)) {
        i2cdrvClearDMA(i2c);
        i2cNotifyClient(i2c);

        i2cTryNextMessage(i2c);
    }
    if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTEFlag)) {
        DMA_ClearITPendingBit(i2c->def->dmaRxStream, i2c->def->dmaRxTEFlag);
        i2c->txMessage.status = i2cNack;
        i2cNotifyClient(i2c);
        i2cTryNextMessage(i2c);
    }
}


void __attribute__((used)) I2C1_ER_IRQHandler(void)
{
    i2cdrvErrorIsrHandler(&deckBus);
}

void __attribute__((used)) I2C1_EV_IRQHandler(void)
{
    i2cdrvEventIsrHandler(&deckBus);
}

void __attribute__((used)) DMA1_Stream0_IRQHandler(void)
{
    i2cdrvDmaIsrHandler(&deckBus);
}

void __attribute__((used)) I2C3_ER_IRQHandler(void)
{
    i2cdrvErrorIsrHandler(&sensorsBus);
}

void __attribute__((used)) I2C3_EV_IRQHandler(void)
{
    i2cdrvEventIsrHandler(&sensorsBus);
}

void __attribute__((used)) DMA1_Stream2_IRQHandler(void)
{
    i2cdrvDmaIsrHandler(&sensorsBus);
}

int i2cdevInit(I2C_Dev *dev)
{
    i2cdrvTryToRestartBus(dev);

    dev->isBusFreeSemaphore = xSemaphoreCreateBinaryStatic(
            &dev->isBusFreeSemaphoreBuffer);
    dev->isBusFreeMutex = xSemaphoreCreateMutexStatic(
            &dev->isBusFreeMutexBuffer);

    return true;
}

//////////////////////////////////////////////////////////////////////////////

bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data)
{
    I2cMessage message;

    i2cdrvCreateMessage(&message, devAddress, i2cRead, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint16_t len, const uint8_t *data)
{
    I2cMessage message;

    i2cdrvCreateMessage(&message, devAddress, i2cWrite, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
        uint16_t len, uint8_t *data)
{
    I2cMessage message;

    i2cdrvCreateMessage16(&message, devAddress, false, memAddress,
            i2cRead, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
        uint16_t len, const uint8_t *data)
{
    I2cMessage message;

    i2cdrvCreateMessage16(&message, devAddress, false, memAddress,
            i2cWrite, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
        uint16_t len, uint8_t *data)
{
    I2cMessage message;

    i2cdrvCreateMessage16(&message, devAddress, true, memAddress,
            i2cRead, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
        uint16_t len, const uint8_t *data)
{
    I2cMessage message;

    i2cdrvCreateMessage16(&message, devAddress, true, memAddress,
            i2cWrite, len, data);

    return i2cdrvMessageTransfer(dev, &message);
}

//////////////////////////////////////////////////////////////////////////////

bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
        uint8_t *data)
{
    return i2cdevReadReg8(dev, devAddress, memAddress, 1, data);
}

bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
        uint8_t data)
{
    return i2cdevWriteReg8(dev, devAddress, memAddress, 1, &data);
}
