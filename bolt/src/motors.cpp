
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

#include <stm32fxxx.h>

#include <time.h>
#include <motors_api.h>

enum {
    MOTOR_M1,
    MOTOR_M2,
    MOTOR_M3,
    MOTOR_M4,
};

typedef struct {

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

// The following defines gives a PWM of 8 bits at ~328KHz for a sysclock of
// 168MHz CF2 PWM ripple is filtered better at 328kHz. At 168kHz the NCP702
// regulator is affected.
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

/**
 * *WARNING* Make sure the brushless driver is configured correctly as on the
 * Crazyflie with normal brushed motors connected they can turn on at full
 * speed when it is powered on!
 *
 * Generates a PWM wave at 2000 Hz update rate, with 125 - 250us high pulse,
 * using the timer.
 */
#define BLMC_PERIOD 0.0005   // 0.5ms = 2000Hz
#define MOTORS_HIGH_PERIOD_ZERO  0.000125 // 125us for zero throttle

#define MOTORS_BL_PWM_PRESCALE_RAW   (uint32_t)((TIM_CLOCK_HZ/0xFFFF) * BLMC_PERIOD + 1) // +1 is to not end up above 0xFFFF in the end
#define MOTORS_BL_PWM_CNT_FOR_PERIOD (uint32_t)(TIM_CLOCK_HZ * BLMC_PERIOD / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_CNT_FOR_HIGH   (uint32_t)(TIM_CLOCK_HZ * MOTORS_HIGH_PERIOD_ZERO / MOTORS_BL_PWM_PRESCALE_RAW)
#define MOTORS_BL_PWM_PERIOD         MOTORS_BL_PWM_CNT_FOR_PERIOD
#define MOTORS_BL_PWM_PRESCALE       (uint16_t)(MOTORS_BL_PWM_PRESCALE_RAW - 1)

// Bolt M1, PA1, TIM2_CH2, Brushless config
static const MotorPerifDef MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP =
{
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
 * Brushless motors mapped to the Bolt 1.1 PWM outputs.
 */
const MotorPerifDef * motorMap[4] = {
    &MOTORS_PA1_TIM2_CH2_BRUSHLESS_PP,
    &MOTORS_PB11_TIM2_CH4_BRUSHLESS_PP,
    &MOTORS_PA15_TIM2_CH1_BRUSHLESS_PP,
    &MOTORS_PB10_TIM2_CH3_BRUSHLESS_PP
};


const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

static bool didInit = false;
static uint64_t lastCycleTime;
static uint32_t cycleTime;

static uint16_t motorsBLConv16ToBits(uint16_t bits)
{
    return (MOTORS_BL_PWM_CNT_FOR_HIGH +
            ((bits * MOTORS_BL_PWM_CNT_FOR_HIGH) / 0xFFFF));
}

static uint32_t motor_ratios[] = {0, 0, 0, 0};  // actual PWM signals

// Ithrust is thrust mapped for 65536 <==> 60 grams
static void setRatio(uint32_t id, uint16_t ithrust)
{
    if (didInit) {

        uint16_t ratio = ithrust;

        motor_ratios[id] = ratio;

        motorMap[id]->setCompare(motorMap[id]->tim, motorsBLConv16ToBits(ratio));

        if (id == MOTOR_M1)
        {
            uint64_t currTime = micros();
            cycleTime = currTime - lastCycleTime;
            lastCycleTime = currTime;
        }
    }
}


//////////////////////////////////////////////////////////////////////////////

int motorsGetRatio(uint32_t id)
{
    return motor_ratios[id];
}

void motorsStop()
{
    for (uint8_t i = 0; i < 4; i++) {
        setRatio(MOTORS[i], 0);
    }
}

// Initialization. Will set all motor ratios to 0%
void motorsInit(void)
{
    if (didInit) {
        // First to init will configure it
        return;
    }

    //Init structures
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    for (uint8_t i=0; i<4; i++) {

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
        MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort,
                motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

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

    // Start the timers
    for (int i = 0; i < 4; i++) {
        TIM_Cmd(motorMap[i]->tim, ENABLE);
    }

    didInit = true;

    // Output zero power
    motorsStop();
}

bool motorsTest(void)
{
    return didInit;
}

void motorsSetRatios(const uint16_t ratios[])
{
  setRatio(MOTOR_M1, ratios[0]);
  setRatio(MOTOR_M2, ratios[1]);
  setRatio(MOTOR_M3, ratios[2]);
  setRatio(MOTOR_M4, ratios[3]);
}
