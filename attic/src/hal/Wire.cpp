#include <string.h>

#include <cfassert.h>

#include "i2cdev.h"
#include "nvicconf.h"
#include "time.h"

#include <Wire.h>

static const uint32_t I2C_DEFAULT_DECK_CLOCK_SPEED = 400000;
static const uint8_t  I2C_SLAVE_ADDRESS7 = 0x30;
static const uint32_t I2C_MAX_RETRIES = 2;
static const uint32_t I2C_MESSAGE_TIMEOUT =  M2T(1000);
static const uint32_t I2CDEV_CLK_TS = 10;
static const uint16_t I2C_NO_INTERNAL_ADDRESS = 0xFFFF;

static void gpioWaitForHigh(GPIO_TypeDef *gpio, uint16_t pin, uint16_t timeout_us)
{
    uint64_t start = micros();
    while (GPIO_ReadInputDataBit(gpio, pin) == Bit_RESET && micros() - start <= timeout_us)
    {
    }
}

static const I2cDef busDef =
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

I2C_Dev bus =
{
    .def = &busDef,
};

static void devUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, 
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


static void startTransfer(I2C_Dev *i2c)
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

static void i2cTryNextMessage(I2C_Dev* i2c)
{
    i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

static void i2cNotifyClient(I2C_Dev* i2c)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2c->isBusFreeSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void dmaSetupBus(I2C_Dev* i2c)
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


static void tryToRestartBus(I2C_Dev* i2c)
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

    devUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, 
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

    dmaSetupBus(i2c);
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

static void clearDMA(I2C_Dev* i2c)
{
    DMA_Cmd(i2c->def->dmaRxStream, DISABLE);
    DMA_ClearITPendingBit(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag);
    I2C_DMACmd(i2c->def->i2cPort, DISABLE);
    I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
    DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, DISABLE);
}

static bool messageTransfer(I2C_Dev* i2c, I2cMessage* message)
{
    bool status = false;

    xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); 

    memcpy((char*)&i2c->txMessage, (char*)message, sizeof(I2cMessage));

    startTransfer(i2c);

    if (xSemaphoreTake(i2c->isBusFreeSemaphore, I2C_MESSAGE_TIMEOUT) == pdTRUE) {
        if (i2c->txMessage.status == i2cAck) {
            status = true;
        }
    }
    else {
        clearDMA(i2c);
        tryToRestartBus(i2c);

    }
    xSemaphoreGive(i2c->isBusFreeMutex);

    return status;
}

static void eventIsrHandler(I2C_Dev* i2c)
{
    uint16_t SR1;
    uint16_t SR2;


    SR1 = i2c->def->i2cPort->SR1;


    if (SR1 & I2C_SR1_SB) {
        i2c->messageIndex = 0;

        if (i2c->txMessage.direction == i2cWrite ||
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
        if (i2c->txMessage.direction == i2cWrite ||
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
        else { if (i2c->txMessage.messageLength == 1)
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
            if (i2c->messageIndex == i2c->txMessage.messageLength)
            {
                i2cNotifyClient(i2c);

                i2cTryNextMessage(i2c);
            }
        }


        while (i2c->def->i2cPort->CR1 & 0x0100) { ; }
    }

    else if (SR1 & I2C_SR1_RXNE) {
        i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
        if (i2c->messageIndex == i2c->txMessage.messageLength)
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
            if (i2c->messageIndex == i2c->txMessage.messageLength) {

                I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
                __DMB();
            }
        }
    }
}

static void errorIsrHandler(I2C_Dev* i2c)
{
    if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_AF)) {
        if (i2c->txMessage.nbrOfRetries-- > 0) {

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

static void dmaIsrHandler(I2C_Dev* i2c)
{
    if (DMA_GetFlagStatus(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag)) {
        clearDMA(i2c);
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
//////////////////////////////////////////////////////////////////////////////

void TwoWire::begin()
{
    (void)dmaIsrHandler;
    (void)errorIsrHandler;
    (void)eventIsrHandler;

    tryToRestartBus(&bus);

    bus.isBusFreeSemaphore = xSemaphoreCreateBinaryStatic(
            &bus.isBusFreeSemaphoreBuffer);

    bus.isBusFreeMutex = xSemaphoreCreateMutexStatic(
            &bus.isBusFreeMutexBuffer);
}

void TwoWire::beginTransmission(const uint8_t addr)
{
    _addr = addr;
    _bufidx = 0;
}

void TwoWire::write(const uint8_t value)
{
    _buffer[_bufidx++] = value;
}

uint8_t TwoWire::endTransmission(const uint8_t stop)
{
    I2cMessage message = {};

    createMessage(&message, _addr, i2cWrite, _bufidx, _buffer);

    return messageTransfer(&bus, &message) ? 0 : 4;
}

size_t TwoWire::requestFrom(
        const uint8_t address, const size_t size, bool sendStop)
{
    _bufidx = 0;

    I2cMessage message = {};

    createMessage(&message, address, i2cRead, size, _buffer);

    return messageTransfer(&bus, &message) ? size : 0;
}

uint8_t TwoWire::read()
{
    return _buffer[_bufidx++];
}


