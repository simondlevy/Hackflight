/**
 *
 * Copyright (C) 2025 Simon D. Levy
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
 */

static const uint8_t LED_PIN = PB5;

#define RESET_NONE                      0
#define RESET_BOOTLOADER_REQUEST_ROM    1

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;


typedef enum {
    PERSISTENT_OBJECT_MAGIC = 0,
    PERSISTENT_OBJECT_HSE_VALUE,
    PERSISTENT_OBJECT_OVERCLOCK_LEVEL,
    PERSISTENT_OBJECT_RESET_REASON,
    PERSISTENT_OBJECT_RTC_HIGH,           // high 32 bits of rtcTime_t
    PERSISTENT_OBJECT_RTC_LOW,            // low 32 bits of rtcTime_t
    PERSISTENT_OBJECT_COUNT,
} persistentObjectId_e;

static void persistentObjectWrite(persistentObjectId_e id, uint32_t value)
{
    RTC_HandleTypeDef rtcHandle = { .Instance = RTC };

    HAL_RTCEx_BKUPWrite(&rtcHandle, id, value);
}


static void blinkLed(const uint32_t delay_msec)
{
    digitalWrite(LED_PIN, LOW);
    delay(delay_msec);
    digitalWrite(LED_PIN, HIGH);
    delay(delay_msec);
}

/* USER CODE BEGIN 4 */
#define BOOT_ADDR	0x1FFFF000	// my MCU boot code base address
#define	MCU_IRQS	70u	// no. of NVIC IRQ inputs

struct boot_vectable_ {
    uint32_t Initial_SP;
    void (*Reset_Handler)(void);
};

#define BOOTVTAB	((struct boot_vectable_ *)BOOT_ADDR)

static void JumpToBootloader(void)
{
	__disable_irq();

	SysTick->CTRL = 0;

	HAL_RCC_DeInit();

	for (uint8_t i = 0; i < (MCU_IRQS + 31u) / 32; i++)
	{
		NVIC->ICER[i]=0xFFFFFFFF;
		NVIC->ICPR[i]=0xFFFFFFFF;
	}

	__enable_irq();

	__set_MSP(BOOTVTAB->Initial_SP);

    SYSCFG->MEMRMP = 0x01;

	BOOTVTAB->Reset_Handler();
}


void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
}


void loop() 
{

    blinkLed(500);

    if (Serial.available() && Serial.read() == 'R') {
        JumpToBootloader();
    }
}
