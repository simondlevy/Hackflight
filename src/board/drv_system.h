#pragma once

#define BKP_SOFTRESET (0x50F7B007)

#define GYRO
#define ACC
#define LED0
#define LED1
#define INVERTER

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define INV_PIN     Pin_2 // PB2 (BOOT1) abused as inverter select GPIO
#define INV_GPIO    GPIOB

#ifdef LED0
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);
#else
#define LED0_TOGGLE
#define LED0_OFF
#define LED0_ON
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);
#else
#define LED1_TOGGLE
#define LED1_OFF
#define LED1_ON
#endif

#ifdef INV_GPIO
#define INV_OFF                  digitalLo(INV_GPIO, INV_PIN);
#define INV_ON                   digitalHi(INV_GPIO, INV_PIN);
#else
#define INV_OFF                 ;
#define INV_ON                  ;
#endif


void systemInit(int hwrev);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);

// Backup SRAM R/W
uint32_t rccReadBkpDr(void);
void rccWriteBkpDr(uint32_t value);

// failure
void failureMode(uint8_t mode);

// bootloader/IAP
void systemReset(bool toBootloader);

// current crystal frequency - 8 or 12MHz
extern uint32_t hse_value;
