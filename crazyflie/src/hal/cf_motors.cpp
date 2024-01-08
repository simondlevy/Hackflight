
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <stdbool.h>

#include <stm32fxxx.h>

#include <free_rtos.h>
#include <task.h>

#include <autoconf.h>
#include <cfassert.h>

#include <arduino/time.h>

#include <platform/platform.h>

#include <console.h>
#include <nvicconf.h>

#include "cf_motors.h"

// Test defines
#define MOTORS_TEST_RATIO         (uint16_t)(0.2*(1<<16))
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150

// The following defines gives a PWM of 8 bits at ~328KHz for a sysclock of 168MHz
// CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702 regulator is affected.
#define TIM_CLOCK_HZ 84000000
#define MOTORS_PWM_BITS           8
#define MOTORS_PWM_PERIOD         ((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       0
#define MOTORS_TIM_BEEP_CLK_FREQ  (84000000L / 5)
#define MOTORS_POLARITY           TIM_OCPolarity_High

// Abstraction of ST lib functions
#define MOTORS_GPIO_MODE          GPIO_Mode_AF
#define MOTORS_RCC_GPIO_CMD       RCC_AHB1PeriphClockCmd
#define MOTORS_RCC_TIM_CMD        RCC_APB1PeriphClockCmd
#define MOTORS_TIM_DBG_CFG        DBGMCU_APB2PeriphConfig
#define MOTORS_GPIO_AF_CFG(a,b,c) GPIO_PinAFConfig(a,b,c)

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT125
/**
 * *WARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
 * brushed motors connected they can turn on at full speed when it is powered on!
 *
 * Generates a PWM wave at 2000 Hz update rate, with 125 - 250us high pulse, using the timer.
 */
#define BLMC_PERIOD 0.0005   // 0.5ms = 2000Hz
#define MOTORS_HIGH_PERIOD_ZERO  0.000125 // 125us for zero throttle

#define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
#define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * MOTORS_HIGH_PERIOD_ZERO / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
#define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)
#define MOTORS_BL_POLARITY           TIM_OCPolarity_Low
#elif defined(CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT42)
/**
 * *WARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
 * brushed motors connected they can turn on at full speed when it is powered on!
 *
 * Generates a PWM wave at 2000 Hz update rate, with 125 - 250us high pulse, using the timer.
 */
#define BLMC_PERIOD 0.000085   // 85us = ~11700Hz
#define MOTORS_HIGH_PERIOD_ZERO  0.000042 // 42us for zero throttle

#define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
#define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * MOTORS_HIGH_PERIOD_ZERO / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
#define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)
#define MOTORS_BL_POLARITY           TIM_OCPolarity_Low
#elif defined(CONFIG_MOTORS_ESC_PROTOCOL_DSHOT)
/**
 * *WARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
 * brushed motors connected they can turn on at full speed when it is powered on!
 *
 */
#ifdef CONFIG_MOTORS_DSHOT_PWM_150KHZ
#define MOTORS_BL_PWM_PERIOD         (TIM_CLOCK_HZ / 150000) // 150kHz bitrate DHSOT150
#endif
#ifdef CONFIG_MOTORS_DSHOT_PWM_300KHZ
#define MOTORS_BL_PWM_PERIOD         (TIM_CLOCK_HZ / 300000) // 300kHz bitrate DHSOT300
#endif
#ifdef CONFIG_MOTORS_DSHOT_PWM_600KHZ
#define MOTORS_BL_PWM_PERIOD         (TIM_CLOCK_HZ / 600000) // 600kHz bitrate DHSOT600
#endif
#define MOTORS_BL_PWM_PRESCALE       (0)
#define MOTORS_BL_POLARITY           TIM_OCPolarity_Low
#define MOTORS_TIM_VALUE_FOR_0       (uint16_t)(MOTORS_BL_PWM_PERIOD * 0.37425)
#define MOTORS_TIM_VALUE_FOR_1       (uint16_t)(MOTORS_BL_PWM_PERIOD * 0.7485)
#define DSHOT_FRAME_SIZE             16
#define DSHOT_DMA_BUFFER_SIZE        17 /* With zero ending  */
#define DSHOT_MIN_THROTTLE           48
#define DSHOT_MAX_THROTTLE           2047
#define DSHOT_RANGE                  (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#define MOTORS_BL_PWM_CNT_FOR_HIGH   1
#else
/**
 * *WARNING* Make sure the brushless driver is configured correctly as on the Crazyflie with normal
 * brushed motors connected they can turn on at full speed when it is powered on!
 *
 * Generates a PWM wave (50 - 400 Hz update rate with 1-2 ms high pulse) using the timer. That way we can use the same
 * base as for the regular PWM driver. This means it will be a PWM with a period of the update rate configured to be high
 * only in the 1-2 ms range.
 */
#define BLMC_PERIOD 0.0025   // 2.5ms = 400Hz
#define MOTORS_HIGH_PERIOD_ZERO  0.001 // 1ms for zero throttle

#define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
#define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * MOTORS_HIGH_PERIOD_ZERO / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
#define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)
#define MOTORS_BL_POLARITY           TIM_OCPolarity_Low
#endif


// Shared with params
bool motorSetEnable;
uint16_t motorPowerSet[] = {0, 0, 0, 0}; // user-requested PWM signals (overrides)

// Shared with logger
uint32_t motor_ratios[] = {0, 0, 0, 0};  // actual PWM signals

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static DMA_InitTypeDef DMA_InitStructureShare;
// Memory buffer for DSHOT bits
static uint32_t dshotDmaBuffer[NBR_OF_MOTORS][DSHOT_DMA_BUFFER_SIZE];
static void motorsDshotDMASetup();
static volatile uint32_t dmaWait;
#endif

// CF2.X connector M1, PA1, TIM2_CH2
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,
};

// CF2.X connector M2, PB11, TIM2_CH4
static const MotorPerifDef MOTORS_PB11_TIM2_CH4_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// CF2.X connector M3, PA15, TIM2_CH1
static const MotorPerifDef MOTORS_PA15_TIM2_CH1_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// CF2.X connector M4, PB9, TIM4_CH4
static const MotorPerifDef MOTORS_PB9_TIM4_CH4_BRUSHED =
{
    .drvType       = BRUSHED,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_9,
    .gpioPinSource = GPIO_PinSource9,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM4,
    .timPerif      = RCC_APB1Periph_TIM4,
    .tim           = TIM4,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM4_STOP,
    .timPeriod     = MOTORS_PWM_PERIOD,
    .timPrescaler  = MOTORS_PWM_PRESCALE,
    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Bolt M1, PA1, TIM2_CH2, Brushless config
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_1,
    .gpioPinSource = GPIO_PinSource1,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOA,
    .gpioPowerswitchPort  = GPIOA,
    .gpioPowerswitchPin   = GPIO_Pin_0,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .DMA_stream    = DMA1_Stream6,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR2,
    .TIM_DMASource = TIM_DMA_CC2,
    .DMA_IRQChannel = DMA1_Stream6_IRQn,

    .setCompare    = TIM_SetCompare2,
    .getCompare    = TIM_GetCapture2,
    .ocInit        = TIM_OC2Init,
    .preloadConfig = TIM_OC2PreloadConfig,

};

// Bolt M2, PB11, TIM2_CH4, Brushless config
static const MotorPerifDef MOTORS_PB11_TIM2_CH4_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_11,
    .gpioPinSource = GPIO_PinSource11,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOB,
    .gpioPowerswitchPort  = GPIOB,
    .gpioPowerswitchPin   = GPIO_Pin_12,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .DMA_stream    = DMA1_Stream7,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR4,
    .TIM_DMASource = TIM_DMA_CC4,
    .DMA_IRQChannel = DMA1_Stream7_IRQn,

    .setCompare    = TIM_SetCompare4,
    .getCompare    = TIM_GetCapture4,
    .ocInit        = TIM_OC4Init,
    .preloadConfig = TIM_OC4PreloadConfig,
};

// Bolt M3, PA15, TIM2_CH1, Brushless config
static const MotorPerifDef MOTORS_PA15_TIM2_CH1_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_15,
    .gpioPinSource = GPIO_PinSource15,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_8,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .DMA_stream    = DMA1_Stream5,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR1,
    .TIM_DMASource = TIM_DMA_CC1,
    .DMA_IRQChannel = DMA1_Stream5_IRQn,

    .setCompare    = TIM_SetCompare1,
    .getCompare    = TIM_GetCapture1,
    .ocInit        = TIM_OC1Init,
    .preloadConfig = TIM_OC1PreloadConfig,
};

// Bolt 1.1 M4, PB10, TIM2_CH3, Brushless config
static const MotorPerifDef MOTORS_PB10_TIM2_CH3_BRUSHLESS_PP =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOB,
    .gpioPort      = GPIOB,
    .gpioPin       = GPIO_Pin_10,
    .gpioPinSource = GPIO_PinSource10,
    .gpioOType     = GPIO_OType_PP,
    .gpioAF        = GPIO_AF_TIM2,
    .gpioPowerswitchPerif = RCC_AHB1Periph_GPIOC,
    .gpioPowerswitchPort  = GPIOC,
    .gpioPowerswitchPin   = GPIO_Pin_15,
    .timPerif      = RCC_APB1Periph_TIM2,
    .tim           = TIM2,
    .timPolarity   = TIM_OCPolarity_High,
    .timDbgStop    = DBGMCU_TIM2_STOP,
    .timPeriod     = MOTORS_BL_PWM_PERIOD,
    .timPrescaler  = MOTORS_BL_PWM_PRESCALE,
    .DMA_stream    = DMA1_Stream1,
    .DMA_Channel   = DMA_Channel_3,
    .DMA_PerifAddr = (uint32_t)&TIM2->CCR3,
    .TIM_DMASource = TIM_DMA_CC3,
    .DMA_IRQChannel = DMA1_Stream1_IRQn,

    .setCompare    = TIM_SetCompare3,
    .getCompare    = TIM_GetCapture3,
    .ocInit        = TIM_OC3Init,
    .preloadConfig = TIM_OC3PreloadConfig,
};

/**
 * Default brushed mapping to M1-M4 connectors.
 */
const MotorPerifDef* motorMapDefaultBrushed[NBR_OF_MOTORS] =
{
    &MOTORS_PA1_TIM2_CH2_BRUSHED,
    &MOTORS_PB11_TIM2_CH4_BRUSHED,
    &MOTORS_PA15_TIM2_CH1_BRUSHED,
    &MOTORS_PB9_TIM4_CH4_BRUSHED
};

/**
 * Brushless motors mapped to the Bolt 1.1 PWM outputs.
 */
const MotorPerifDef* motorMapBolt11Brushless[NBR_OF_MOTORS] =
{
    &MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP,
    &MOTORS_PB11_TIM2_CH4_BRUSHLESS_PP,
    &MOTORS_PA15_TIM2_CH1_BRUSHLESS_PP,
    &MOTORS_PB10_TIM2_CH3_BRUSHLESS_PP
};


const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {440, 880, 698, 587};

static bool didInit = false;
static uint64_t lastCycleTime;
static uint32_t cycleTime;

/* Private functions */

#ifndef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
    return (MOTORS_BL_PWM_CNT_FOR_HIGH + ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}
#endif

static uint16_t motorsConv16ToBits(uint16_t bits)
{
    return ((bits) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

GPIO_InitTypeDef GPIO_PassthroughInput =
{
    .GPIO_Mode = GPIO_Mode_IN,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_OType = GPIO_OType_OD,
    .GPIO_PuPd = GPIO_PuPd_UP
};

GPIO_InitTypeDef GPIO_PassthroughOutput =
{
    .GPIO_Mode = GPIO_Mode_OUT,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_OType = GPIO_OType_PP,
    .GPIO_PuPd = GPIO_PuPd_UP
};

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
static void motorsDshotDMASetup()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* DMA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    // Preparation of common things in DMA setup struct
    DMA_InitStructureShare.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructureShare.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructureShare.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructureShare.DMA_BufferSize = DSHOT_DMA_BUFFER_SIZE;
    DMA_InitStructureShare.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureShare.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructureShare.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructureShare.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructureShare.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructureShare.DMA_Priority = DMA_Priority_High;
    DMA_InitStructureShare.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructureShare.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;

    for (int i = 0; i < NBR_OF_MOTORS; i++)
    {
        DMA_InitStructureShare.DMA_PeripheralBaseAddr = motorMap[i]->DMA_PerifAddr;
        DMA_InitStructureShare.DMA_Memory0BaseAddr = (uint32_t)dshotDmaBuffer[i];
        DMA_InitStructureShare.DMA_Channel = motorMap[i]->DMA_Channel;
        DMA_Init(motorMap[i]->DMA_stream, &DMA_InitStructureShare);

        NVIC_InitStructure.NVIC_IRQChannel = motorMap[i]->DMA_IRQChannel;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_MOTORS_PRI;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
}

static void motorsPrepareDshot(uint32_t id, uint16_t ratio)
{
    uint16_t dshotBits;
    bool dshot_telemetry = false;
    uint16_t dshotRatio;

    ASSERT(id < NBR_OF_MOTORS);

    // Scale 16 -> 11 bits
    dshotRatio = (ratio >> 5);
    // Remove command area of DSHOT
    if (dshotRatio < (DSHOT_MIN_THROTTLE - 1))
    {
        dshotRatio = 0;
    }

    dshotBits = (dshotRatio << 1) | (dshot_telemetry ? 1 : 0);

    // compute checksum
    unsigned cs = 0;
    unsigned csData = dshotBits;

    for (int i = 0; i < 3; i++)
    {
        cs ^=  csData; // xor data by nibbles
        csData >>= 4;
    }

    cs &= 0xf;
    dshotBits = (dshotBits << 4) | cs;

    for(int i = 0; i < DSHOT_FRAME_SIZE; i++)
    {
        dshotDmaBuffer[id][i] = (dshotBits & 0x8000) ? MOTORS_TIM_VALUE_FOR_1 : MOTORS_TIM_VALUE_FOR_0;
        dshotBits <<= 1;
    }
    dshotDmaBuffer[id][16] = 0; // Set to 0 gives low output afterwards

    // Wait for DMA to be free. Can happen at startup but doesn't seem to wait afterwards.
    while(DMA_GetCmdStatus(motorMap[id]->DMA_stream) != DISABLE)
    {
        dmaWait++;
    }
}

/**
 * Unfortunately the TIM2_CH2 (M1) and TIM2_CH4 (M2) share DMA channel 3 request and can't
 * be used at the same time. Solved by running after each other and TIM2_CH2
 * will be started in DMA1_Stream6_IRQHandler. Thus M2 will have a bit of latency.
 */
void motorsBurstDshot()
{

    motorMap[0]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    motorMap[1]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    /* Enable TIM DMA Requests M1*/
    TIM_DMACmd(motorMap[0]->tim, motorMap[0]->TIM_DMASource, ENABLE);
    DMA_ITConfig(motorMap[0]->DMA_stream, DMA_IT_TC, ENABLE);
    DMA_ITConfig(motorMap[1]->DMA_stream, DMA_IT_TC, ENABLE);
    /* Enable DMA TIM Stream */
    DMA_Cmd(motorMap[0]->DMA_stream, ENABLE);

    motorMap[2]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    /* Enable TIM DMA Requests M3*/
    TIM_DMACmd(motorMap[2]->tim, motorMap[2]->TIM_DMASource, ENABLE);
    DMA_ITConfig(motorMap[2]->DMA_stream, DMA_IT_TC, ENABLE);
    /* Enable DMA TIM Stream */
    DMA_Cmd(motorMap[2]->DMA_stream, ENABLE);

    motorMap[3]->DMA_stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
    /* Enable TIM DMA Requests M4*/
    TIM_DMACmd(motorMap[3]->tim, motorMap[3]->TIM_DMASource, ENABLE);
    DMA_ITConfig(motorMap[3]->DMA_stream, DMA_IT_TC, ENABLE);
    /* Enable DMA TIM Stream */
    DMA_Cmd(motorMap[3]->DMA_stream, ENABLE);
}
#endif



#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
extern "C" {
    void __attribute__((used)) DMA1_Stream1_IRQHandler(void)  // M4
    {
        TIM_DMACmd(TIM2, TIM_DMA_CC3, DISABLE);
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
        DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, DISABLE);
    }
    void __attribute__((used)) DMA1_Stream5_IRQHandler(void)  // M3
    {
        TIM_DMACmd(TIM2, TIM_DMA_CC1, DISABLE);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
        DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
    }
    void __attribute__((used)) DMA1_Stream6_IRQHandler(void) // M1
    {
        TIM_DMACmd(TIM2, TIM_DMA_CC2, DISABLE);
        DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
        DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, DISABLE);
        /* Enable TIM DMA Requests M2*/
        TIM_DMACmd(motorMap[1]->tim, motorMap[1]->TIM_DMASource, ENABLE);
        /* Enable DMA TIM Stream */
        DMA_Cmd(motorMap[1]->DMA_stream, ENABLE);
    }
    void __attribute__((used)) DMA1_Stream7_IRQHandler(void)  // M2
    {
        TIM_DMACmd(TIM2, TIM_DMA_CC4, DISABLE);
        DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);
        DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, DISABLE);
    }
}
#endif

// Ithrust is thrust mapped for 65536 <==> 60 grams
static void setRatio(uint32_t id, uint16_t ithrust)
{
    if (didInit) {
        ASSERT(id < NBR_OF_MOTORS);

        uint16_t ratio = ithrust;

        if (motorSetEnable) {
            ratio = motorPowerSet[id];
        }

        motor_ratios[id] = ratio;

        if (motorMap[id]->drvType == BRUSHLESS)
        {
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
            // Prepare DSHOT, firing it will be done synchronously with motorsBurstDshot.
            motorsPrepareDshot(id, ratio);
#else
            motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));
#endif
        }
        else
        {
            motorMap[id]->setCompare(motorMap[id]->tim, motorsConv16ToBits(ratio));
        }

        if (id == MOTOR_M1)
        {
            uint64_t currTime = micros();
            cycleTime = currTime - lastCycleTime;
            lastCycleTime = currTime;
        }
    }
}


//////////////////////////////////////////////////////////////////////////////


// We have data that maps PWM to thrust at different supply voltage levels.
// However, it is not the PWM that drives the motors but the voltage and
// amps (= power). With the PWM it is possible to simulate different
// voltage levels. The assumption is that the voltage used will be an
// procentage of the supply voltage, we assume that 50% PWM will result in
// 50% voltage.
//
//  Thrust (g)    Supply Voltage    PWM (%)     Voltage needed
//  0.0           4.01              0           0
//  1.6           3.98              6.25        0.24875
//  4.8           3.95              12.25       0.49375
//  7.9           3.82              18.75       0.735
//  10.9          3.88              25          0.97
//  13.9          3.84              31.25       1.2
//  17.3          3.80              37.5        1.425
//  21.0          3.76              43.25       1.6262
//  24.4          3.71              50          1.855
//  28.6          3.67              56.25       2.06438
//  32.8          3.65              62.5        2.28125
//  37.3          3.62              68.75       2.48875
//  41.7          3.56              75          2.67
//  46.0          3.48              81.25       2.8275
//  51.9          3.40              87.5        2.975
//  57.9          3.30              93.75       3.09375
//
// To get Voltage needed from wanted thrust we can get the quadratic
// polyfit coefficients using GNU octave:
//
// thrust = [0.0 1.6 4.8 7.9 10.9 13.9 17.3 21.0 ...
//           24.4 28.6 32.8 37.3 41.7 46.0 51.9 57.9]
//
// volts  = [0.0 0.24875 0.49375 0.735 0.97 1.2 1.425 1.6262 1.855 ...
//           2.064375 2.28125 2.48875 2.67 2.8275 2.975 3.09375]
//
// p = polyfit(thrust, volts, 2)
//
// => p = -0.00062390   0.08835522   0.06865956
//
// We will not use the constant term, since we want zero thrust to equal
// zero PWM.
//
// And to get the PWM as a percentage we would need to divide the
// Voltage needed with the Supply voltage.
float motorsCompensateBatteryVoltage(uint32_t id, float iThrust, float supplyVoltage)
{
#ifdef CONFIG_ENABLE_THRUST_BAT_COMPENSATED
    ASSERT(id < NBR_OF_MOTORS);

    if (motorMap[id]->drvType == BRUSHED)
    {
        /*
         * A LiPo battery is supposed to be 4.2V charged, 3.7V mid-charge and 3V
         * discharged.
         *
         * A suitable sanity check for disabling the voltage compensation would be
         * under 2V. That would suggest a damaged battery. This protects against
         * rushing the motors on bugs and invalid voltage levels.
         */
        if (supplyVoltage < 2.0f)
        {
            return iThrust;
        }

        float thrust = (iThrust / 65536.0f) * 60;
        float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
        float ratio = volts / supplyVoltage;
        return UINT16_MAX * ratio;
    }
#endif

    return iThrust;
}

int motorsGetRatio(uint32_t id)
{
    ASSERT(id < NBR_OF_MOTORS);

    return motor_ratios[id];
}

void motorsCheckDshot(void)
{
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
#endif
}

void motorsStop()
{
    for (uint8_t i = 0; i < NBR_OF_MOTORS; i++) {
        setRatio(MOTORS[i], 0);
    }

    motorsCheckDshot();

}

//Initialization. Will set all motors ratio to 0%
void motorsInit(void)
{
    const auto motorMapSelectNew = platformConfigGetMotorMapping();

    int i;
    //Init structures
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    if (didInit)
    {
        // First to init will configure it
        return;
    }

    motorMap = motorMapSelectNew;

    consolePrintf("MTR-DRV: Using %s motor driver\n", 
            motorMap[0]->drvType == BRUSHED ? "brushed" : "brushless");

    for (i = 0; i < NBR_OF_MOTORS; i++)
    {
        //Clock the gpio and the timers
        MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
        MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPowerswitchPerif, ENABLE);
        MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

        // If there is a power switch, as on Bolt, enable power to ESC by
        // switching on mosfet.
        if (motorMap[i]->gpioPowerswitchPin != 0)
        {
            GPIO_StructInit(&GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
            GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPowerswitchPin;
            GPIO_Init(motorMap[i]->gpioPowerswitchPort, &GPIO_InitStructure);
            GPIO_WriteBit(motorMap[i]->gpioPowerswitchPort, 
                    motorMap[i]->gpioPowerswitchPin, (BitAction)1);
        }

        // Configure the GPIO for the timer output
        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = (GPIOOType_TypeDef)motorMap[i]->gpioOType;
        GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
        GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

        //Map timers to alternate functions
        MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

        //Timer configuration
        TIM_TimeBaseStructure.TIM_Period = motorMap[i]->timPeriod;
        TIM_TimeBaseStructure.TIM_Prescaler = motorMap[i]->timPrescaler;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

        // PWM channels configuration (All identical!)
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

        // Configure Output Compare for PWM
        motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
        motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);
    }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsDshotDMASetup();
#endif
    // Start the timers
    for (i = 0; i < NBR_OF_MOTORS; i++)
    {
        TIM_Cmd(motorMap[i]->tim, ENABLE);
    }

    didInit = true;

    // Output zero power
    motorsStop();
}

bool motorsTest(void)
{
    for (uint8_t i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
        if (motorMap[i]->drvType == BRUSHED) {
            setRatio(MOTORS[i], MOTORS_TEST_RATIO);
            vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
            setRatio(MOTORS[i], 0);
            vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
        }
    }

    return didInit;
}

void motorsSetRatios(const uint16_t ratios[])
{
  setRatio(MOTOR_M1, ratios[0]);
  setRatio(MOTOR_M2, ratios[1]);
  setRatio(MOTOR_M3, ratios[2]);
  setRatio(MOTOR_M4, ratios[3]);
}
