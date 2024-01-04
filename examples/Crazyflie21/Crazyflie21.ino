static const uint8_t LED_RED_L_PIN = 8;

static void reboot(void)
{
    __enable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    const uint32_t p = (*((uint32_t *) 0x1FFF0000));
    __set_MSP( p );

    void (*SysMemBootJump)(void);
    SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
    SysMemBootJump();

    NVIC_SystemReset();
}

void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_RED_L_PIN, OUTPUT);
}

void loop(void)
{
    /*
       digitalWrite(LED_RED_L_PIN, HIGH);
       delay(500);
       digitalWrite(LED_RED_L_PIN, LOW);
       delay(500);*/

    while (Serial.available()) {

        if (Serial.read() == 'R') {
            reboot();
        }
    }
}
