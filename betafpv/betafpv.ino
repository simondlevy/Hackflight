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

static const uint8_t LED_PIN = PB2; // PB5;

#define BOOTLOADER_ADDR 0x1FFF0000 // Bootloader start address (refer to AN2606). STM32 family-dependent.

struct bootloader_vectable__t {
    uint32_t stack_pointer;
    void (*reset_handler)(void);
};
#define BOOTLOADER_VECTOR_TABLE	((struct bootloader_vectable__t *)BOOTLOADER_ADDR)

void JumpToBootloader_v4(void) {
    // Deinit HAL and Clocks
    HAL_DeInit();
    HAL_RCC_DeInit();

    // Disable all interrupts
    __disable_irq();

    // Disable Systick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    // Disable interrupts and clear pending ones
    for (size_t i = 0; i < sizeof(NVIC->ICER)/sizeof(NVIC->ICER[0]); i++) {
        NVIC->ICER[i]=0xFFFFFFFF;
        NVIC->ICPR[i]=0xFFFFFFFF;
    }

    // Re-enable interrupts
    __enable_irq();

    // Map Bootloader (system flash) memory to 0x00000000. This is STM32 family dependant.
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    // Set embedded bootloader vector table base offset
    WRITE_REG(SCB->VTOR, SCB_VTOR_TBLOFF_Msk & 0x00000000);

    // Switch to Main Stack Pointer (in case it was using the Process Stack Pointer)
    __set_CONTROL(0);

    // Instruction synchronization barrier
    __ISB();

    // Set Main Stack Pointer to the Bootloader defined value.
    __set_MSP(BOOTLOADER_VECTOR_TABLE->stack_pointer);

    __DSB(); // Data synchronization barrier
    __ISB(); // Instruction synchronization barrier

    // Jump to Bootloader Reset Handler
    BOOTLOADER_VECTOR_TABLE->reset_handler();

    // The next instructions will not be reached
    while (1){}
}

void serialEvent()
{
    if (Serial.available() && Serial.read() == 'R') {
        JumpToBootloader_v4();
    }
}

void setup() 
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
}


void loop() 
{

    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);
    delay(500);

}
