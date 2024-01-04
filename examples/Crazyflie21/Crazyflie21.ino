static const uint8_t LED_RED_L_PIN = PC0;

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
    const auto msec = millis();

    static uint32_t prev;

    if (msec - prev > 1000) {
        prev = msec;
        static uint8_t ledOn;
        digitalWrite(LED_RED_L_PIN, ledOn);
        ledOn = 1 - ledOn;
    }

    while (Serial.available()) {

        if (Serial.read() == 'R') {
            reboot();
        }
    }
}
