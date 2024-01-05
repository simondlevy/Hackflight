#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <stm32fxxx.h>

#include <motors.h>

#define NBR_OF_MOTORS 4

typedef struct {

  motorsDrvType drvType;
  uint32_t      gpioPerif;
  GPIO_TypeDef* gpioPort;
  uint16_t      gpioPin;
  uint16_t      gpioPinSource;
  uint32_t      gpioOType;
  uint8_t       gpioAF;
  uint32_t      gpioPowerswitchPerif;
  GPIO_TypeDef* gpioPowerswitchPort;
  uint16_t      gpioPowerswitchPin;
  uint32_t      timPerif;
  TIM_TypeDef*  tim;
  uint16_t      timPolarity;
  uint32_t      timDbgStop;
  uint32_t      timPeriod;
  uint16_t      timPrescaler;
  DMA_Stream_TypeDef *DMA_stream;
  uint32_t      DMA_Channel;
  uint32_t      DMA_PerifAddr;
  uint16_t      TIM_DMASource;
  uint8_t       DMA_IRQChannel;

  /* Function pointers */
  void (*setCompare)(TIM_TypeDef* TIMx, uint32_t Compare);
  uint32_t (*getCompare)(TIM_TypeDef* TIMx);
  void (*ocInit)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void (*preloadConfig)(TIM_TypeDef* TIMx, uint16_t TIM_OCPreload);

} MotorPerifDef;

extern const uint16_t testsound[NBR_OF_MOTORS];
