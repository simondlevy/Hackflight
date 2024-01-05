#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <free_rtos.h>
#include <semphr.h>
#include <queue.h>

#include <stm32fxxx.h>


#define I2C1_DEV  &deckBus
#define I2C3_DEV  &sensorsBus

typedef enum
{
    i2cAck,
    i2cNack
} I2cStatus;

typedef enum
{
    i2cWrite,
    i2cRead
} I2cDirection;


typedef struct _I2cMessage
{
    uint32_t         messageLength;		  
    uint8_t          slaveAddress;		 
    uint8_t          nbrOfRetries;      
    I2cDirection     direction;      
    I2cStatus        status;          
    xQueueHandle     clientQueue;     
    bool             isInternal16bit; 
    uint16_t         internalAddress; 
    uint8_t          *buffer;         
} I2cMessage;

typedef struct
{
    I2C_TypeDef*        i2cPort;
    uint32_t            i2cPerif;
    uint32_t            i2cEVIRQn;
    uint32_t            i2cERIRQn;
    uint32_t            i2cClockSpeed;
    uint32_t            gpioSCLPerif;
    GPIO_TypeDef*       gpioSCLPort;
    uint32_t            gpioSCLPin;
    uint32_t            gpioSCLPinSource;
    uint32_t            gpioSDAPerif;
    GPIO_TypeDef*       gpioSDAPort;
    uint32_t            gpioSDAPin;
    uint32_t            gpioSDAPinSource;
    uint32_t            gpioAF;
    uint32_t            dmaPerif;
    uint32_t            dmaChannel;
    DMA_Stream_TypeDef* dmaRxStream;
    uint32_t            dmaRxIRQ;
    uint32_t            dmaRxTCFlag;
    uint32_t            dmaRxTEFlag;

} I2cDef;

typedef struct
{
    const I2cDef *def;                    
    I2cMessage txMessage;                 
    uint32_t messageIndex;                
    uint32_t nbrOfretries;                
    SemaphoreHandle_t isBusFreeSemaphore; 
    StaticSemaphore_t isBusFreeSemaphoreBuffer;
    SemaphoreHandle_t isBusFreeMutex;    
    StaticSemaphore_t isBusFreeMutexBuffer;
    DMA_InitTypeDef DMAStruct;          
} I2cDrv;

extern I2cDrv deckBus;
extern I2cDrv sensorsBus;

typedef I2cDrv    I2C_Dev;

#ifdef __cplusplus
extern "C" {
#endif

    int i2cdevInit(I2C_Dev *dev);

    // hal_lh_bootloader.c
    bool i2cdevRead(I2C_Dev *dev, uint8_t devAddress, uint16_t len, uint8_t *data);
    bool i2cdevWrite(I2C_Dev *dev, uint8_t devAddress, uint16_t len, const uint8_t *data);

    // tasks/sensors.cpp, deck/flowdeck/bstdr_comm_support.c
    bool i2cdevReadReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
            uint16_t len, uint8_t *data);
    bool i2cdevWriteReg8(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
            uint16_t len, const uint8_t *data);

    // deck/zranger/vl53l1x_crazyflie.cpp, hal/eeprom.c
    bool i2cdevReadReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
            uint16_t len, uint8_t *data);
    bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t devAddress, uint16_t memAddress,
            uint16_t len, const uint8_t *data);

    // hal/pca95x4.c,pca9555.c
    bool i2cdevReadByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
            uint8_t *data);
    bool i2cdevWriteByte(I2C_Dev *dev, uint8_t devAddress, uint8_t memAddress,
            uint8_t data);

#ifdef __cplusplus
}
#endif
