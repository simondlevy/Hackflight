#include <string.h>

#include <free_rtos.h>
#include <semphr.h>
#include <task.h>

#include <stm32fxxx.h>

#include "../nvicconf.h"

#include "spi2.h"

#define SPI_TX_DMA_STREAM       DMA1_Stream4
#define SPI_TX_DMA_IRQ          DMA1_Stream4_IRQn
#define SPI_TX_DMA_IRQHandler   DMA1_Stream4_IRQHandler
#define SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF4

#define SPI2_RX_DMA_STREAM       DMA1_Stream3
#define SPI2_RX_DMA_IRQ          DMA1_Stream3_IRQn
#define SPI2_RX_DMA_IRQHandler   DMA1_Stream3_IRQHandler
#define SPI2_RX_DMA_CHANNEL      DMA_Channel_0
#define SPI2_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define SPI2_CLK                 RCC_APB1Periph_SPI2

#define SPI2_SCK_PIN             GPIO_Pin_13
#define SPI2_MISO_PIN            GPIO_Pin_14
#define SPI2_MOSI_PIN            GPIO_Pin_15

#define SPI2_SCK_SRC             GPIO_PinSource13
#define SPI2_MISO_SRC            GPIO_PinSource14
#define SPI2_MOSI_SRC            GPIO_PinSource15

#define SPI2_CLK_INIT            RCC_AHB1PeriphClockCmd

#define SPI2_GPIO_PERIF          RCC_AHB1Periph_GPIOB

#define SPI2_AF                  GPIO_AF_SPI2

#define SPI2_DMA                 DMA1

#define SPI2_DMA_CLK             RCC_AHB1Periph_DMA1

#define SPI_MAX_DMA_TRANSACTION_SIZE    15
#define SPI2_GPIO_PORT GPIOB

static uint8_t spiTxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static uint8_t spiRxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];

static xSemaphoreHandle spiTxDMAComplete;
static StaticSemaphore_t spiTxDMACompleteBuffer;
static xSemaphoreHandle spiRxDMAComplete;
static StaticSemaphore_t spiRxDMACompleteBuffer;

void spi2_begin(void)
{
  // 1 ---------------------------------------------------------------------
  spiTxDMAComplete = xSemaphoreCreateBinaryStatic(&spiTxDMACompleteBuffer);
  spiRxDMAComplete = xSemaphoreCreateBinaryStatic(&spiRxDMACompleteBuffer);

  GPIO_InitTypeDef GPIO_InitStructure = {};

  // 4 ---------------------------------------------------------------------
  RCC_AHB1PeriphClockCmd(SPI2_GPIO_PERIF, ENABLE);

  RCC_APB1PeriphClockCmd(SPI2_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN |  SPI2_MOSI_PIN;

  // A ---------------------------------------------------------------------
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);

  // 2 ---------------------------------------------------------------------
  GPIO_PinAFConfig(SPI2_GPIO_PORT, SPI2_SCK_SRC, SPI2_AF);
  GPIO_PinAFConfig(SPI2_GPIO_PORT, SPI2_MISO_SRC, SPI2_AF);
  GPIO_PinAFConfig(SPI2_GPIO_PORT, SPI2_MOSI_SRC, SPI2_AF);

  SPI_Cmd(SPI2, DISABLE);
  SPI_I2S_DeInit(SPI2);

  // B ---------------------------------------------------------------------
  SPI_InitTypeDef  SPI_InitStructure = {};
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //~10.5 MHz
  SPI_Init(SPI2, &SPI_InitStructure);
  
  SPI_Cmd(SPI2, ENABLE);

  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  SPI2_CLK_INIT(SPI2_DMA_CLK, ENABLE);

  // 3 ---------------------------------------------------------------------
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later
  DMA_InitStructure.DMA_Channel = SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(SPI_TX_DMA_STREAM, &DMA_InitStructure);
  DMA_InitStructure.DMA_Channel = SPI2_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(SPI2_RX_DMA_STREAM, DISABLE);
  DMA_Init(SPI2_RX_DMA_STREAM, &DMA_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SPI_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);
}

void spi2_send_byte(uint8_t byte)
{
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

  SPI_I2S_SendData(SPI2, byte);

  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

  SPI_I2S_ReceiveData(SPI2);
}

void spi2_dma_read(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  SPI_Cmd(SPI2, DISABLE);

  spiTxBuffer[0] = reg_addr;
  SPI_TX_DMA_STREAM->M0AR = (uint32_t)&spiTxBuffer[0];
  SPI_TX_DMA_STREAM->NDTR = len + 1;

  SPI2_RX_DMA_STREAM->M0AR = (uint32_t)&spiRxBuffer[0];
  SPI2_RX_DMA_STREAM->NDTR = len + 1;

  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI2_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  DMA_ClearFlag(SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|
                DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
  DMA_ClearFlag(SPI2_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|
                DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

  DMA_Cmd(SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(SPI2_RX_DMA_STREAM,ENABLE);

  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

  SPI_Cmd(SPI2, ENABLE);

  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  memcpy(reg_data, &spiRxBuffer[1], len);
}

extern "C" {

void __attribute__((used)) SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(SPI_TX_DMA_STREAM, SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(SPI_TX_DMA_STREAM,SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_TX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiTxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

void __attribute__((used)) SPI2_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI2_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(SPI2_RX_DMA_STREAM, SPI2_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(SPI2_RX_DMA_STREAM, SPI2_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(SPI2_RX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiRxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

} // extern "C"
