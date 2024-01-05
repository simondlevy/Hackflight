#include "SPI.h"

#include <free_rtos.h>
#include <semphr.h>

#define SPI1_TX_DMA_STREAM       DMA2_Stream5
#define SPI1_TX_DMA_IRQ          DMA2_Stream5_IRQn
#define SPI1_TX_DMA_IRQHandler   DMA2_Stream5_IRQHandler
#define SPI1_TX_DMA_CHANNEL      DMA_Channel_3
#define SPI1_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF5

#define SPI1_RX_DMA_STREAM       DMA2_Stream0
#define SPI1_RX_DMA_IRQ          DMA2_Stream0_IRQn
#define SPI1_RX_DMA_IRQHandler   DMA2_Stream0_IRQHandler
#define SPI1_RX_DMA_CHANNEL      DMA_Channel_3
#define SPI1_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF0

#define SPI1_CLK                 RCC_APB2Periph_SPI1

#define SPI1_SCK_PIN             GPIO_Pin_5
#define SPI1_MISO_PIN            GPIO_Pin_6
#define SPI1_MOSI_PIN            GPIO_Pin_7

#define SPI1_SCK_SRC             GPIO_PinSource5
#define SPI1_MISO_SRC            GPIO_PinSource6
#define SPI1_MOSI_SRC            GPIO_PinSource7

#define SPI1_CLK_INIT            RCC_APB2PeriphClockCmd
#define SPI1_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define SPI1_GPIO_PORT           GPIOA

#define SPI1_GPIO_PERIF          RCC_AHB1Periph_GPIOA

#define SPI1_AF                  GPIO_AF_SPI1

#define SPI1_DMA                 DMA2

#define SPI1_DMA_CLK             RCC_AHB1Periph_DMA2

static void spi1_begin(void);

static void spi1_begin_transaction(
        const uint16_t baudRatePrescaler, const spi_mode_t spiMode);

static bool spi1_exchange(size_t length, const uint8_t * data);

static SemaphoreHandle_t spiMutex;

void SPIClass::begin(void)
{
    spi1_begin();
}

void SPIClass::beginTransaction(const SPISettings & settings)
{
    // Chose baud rate scaler based on 84MHz peripheral clock
    const uint16_t baudRatePrescaler = 
          settings._rate >= 21000000 ? 8
        : settings._rate >= 11500000 ? 16 
        : settings._rate >= 5250000 ? 24
        : settings._rate >= 2625000 ? 32
        : 40;

    spi1_begin_transaction(baudRatePrescaler, settings._mode);
}

void SPIClass::transfer(uint8_t * data, size_t size)
{
    spi1_exchange(size, data);
}

void SPIClass::endTransaction(void)
{
    xSemaphoreGive(spiMutex);
}

//////////////////////////////////////////////////////////////////////////////

#include <stm32fxxx.h>

#include <cfassert.h>
#include <config.h>
#include <nvicconf.h>

SemaphoreHandle_t txComplete;
SemaphoreHandle_t rxComplete;

static void spi1_begin(void)
{
    // 1 ---------------------------------------------------------------------
    txComplete = xSemaphoreCreateBinary();
    rxComplete = xSemaphoreCreateBinary();
    spiMutex = xSemaphoreCreateMutex();

    SPI1_DMA_CLK_INIT(SPI1_DMA_CLK, ENABLE);
    SPI1_CLK_INIT(SPI1_CLK, ENABLE);

    // 4 ---------------------------------------------------------------------
    RCC_AHB1PeriphClockCmd(SPI1_GPIO_PERIF, ENABLE);

    // 2 ---------------------------------------------------------------------
    GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_SCK_SRC, SPI1_AF);
    GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_MISO_SRC, SPI1_AF);
    GPIO_PinAFConfig(SPI1_GPIO_PORT, SPI1_MOSI_SRC, SPI1_AF);

    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 3 ---------------------------------------------------------------------
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_BufferSize = 0; // set later
    DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later
    DMA_InitStructure.DMA_Channel = SPI1_TX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_Cmd(SPI1_TX_DMA_STREAM,DISABLE);
    DMA_Init(SPI1_TX_DMA_STREAM, &DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = SPI1_RX_DMA_CHANNEL;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_Cmd(SPI1_RX_DMA_STREAM,DISABLE);
    DMA_Init(SPI1_RX_DMA_STREAM, &DMA_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SPI_PRI;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = SPI1_TX_DMA_IRQ;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = SPI1_RX_DMA_IRQ;
    NVIC_Init(&NVIC_InitStructure);
}

static void spi1_begin_transaction(
        const uint16_t baudRatePrescaler,
        const spi_mode_t spiMode)
{
    GPIO_InitTypeDef GPIO_InitStructure = {};

    // A ---------------------------------------------------------------------
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    if (spiMode == SPI_MODE3) {
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    }
    else {
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    }

    GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
    GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  SPI1_MOSI_PIN;
    GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  SPI1_MISO_PIN;
    GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStructure);

    xSemaphoreTake(spiMutex, portMAX_DELAY);


    SPI_I2S_DeInit(SPI1);

    // B ---------------------------------------------------------------------
    SPI_InitTypeDef  SPI_InitStructure = {};
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

    if (spiMode == SPI_MODE3) {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }

    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

    SPI_InitStructure.SPI_BaudRatePrescaler = baudRatePrescaler;
    SPI_Init(SPI1, &SPI_InitStructure);
}


static bool spi1_exchange(size_t length, const uint8_t * data)
{
    ASSERT_DMA_SAFE(data);

    // DMA already configured, just need to set memory addresses
    SPI1_TX_DMA_STREAM->M0AR = (uint32_t)data;
    SPI1_TX_DMA_STREAM->NDTR = length;

    SPI1_RX_DMA_STREAM->M0AR = (uint32_t)data;
    SPI1_RX_DMA_STREAM->NDTR = length;

    // Enable SPI DMA Interrupts
    DMA_ITConfig(SPI1_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
    DMA_ITConfig(SPI1_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

    // Clear DMA Flags
    DMA_ClearFlag(SPI1_TX_DMA_STREAM, 
            DMA_FLAG_FEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_TEIF5|DMA_FLAG_HTIF5|DMA_FLAG_TCIF5);
    DMA_ClearFlag(SPI1_RX_DMA_STREAM, 
            DMA_FLAG_FEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_TEIF0|DMA_FLAG_HTIF0|DMA_FLAG_TCIF0);

    // Enable DMA Streams
    DMA_Cmd(SPI1_TX_DMA_STREAM,ENABLE);
    DMA_Cmd(SPI1_RX_DMA_STREAM,ENABLE);

    // Enable SPI DMA requests
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

    // Enable peripheral
    SPI_Cmd(SPI1, ENABLE);

    // Wait for completion
    bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
        && (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

    // Disable peripheral
    SPI_Cmd(SPI1, DISABLE);
    return result;
}

extern "C" {

    void __attribute__((used)) SPI1_TX_DMA_IRQHandler(void)
    {
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

        // Stop and cleanup DMA stream
        DMA_ITConfig(SPI1_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
        DMA_ClearITPendingBit(SPI1_TX_DMA_STREAM, SPI1_TX_DMA_FLAG_TCIF);

        // Clear stream flags
        DMA_ClearFlag(SPI1_TX_DMA_STREAM,SPI1_TX_DMA_FLAG_TCIF);

        // Disable SPI DMA requests
        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

        // Disable streams
        DMA_Cmd(SPI1_TX_DMA_STREAM,DISABLE);

        // Give the semaphore, allowing the SPI transaction to complete
        xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD();
        }
    }

    void __attribute__((used)) SPI1_RX_DMA_IRQHandler(void)
    {
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

        // Stop and cleanup DMA stream
        DMA_ITConfig(SPI1_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
        DMA_ClearITPendingBit(SPI1_RX_DMA_STREAM, SPI1_RX_DMA_FLAG_TCIF);

        // Clear stream flags
        DMA_ClearFlag(SPI1_RX_DMA_STREAM,SPI1_RX_DMA_FLAG_TCIF);

        // Disable SPI DMA requests
        SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

        // Disable streams
        DMA_Cmd(SPI1_RX_DMA_STREAM,DISABLE);

        // Give the semaphore, allowing the SPI transaction to complete
        xSemaphoreGiveFromISR(rxComplete, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD();
        }
    }

} // extern "C"

