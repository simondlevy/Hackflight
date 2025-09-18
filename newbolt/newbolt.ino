#include <STM32FreeRTOS.h>

const uint8_t LED_PIN = PC0;

SemaphoreHandle_t sem;

static void Thread1(void* arg) {
  UNUSED(arg);
  while (1) {

    // Wait for signal from thread 2.
    xSemaphoreTake(sem, portMAX_DELAY);

    // Turn LED off.
    digitalWrite(LED_PIN, LOW);
  }
}

static void Thread2(void* arg) {
  UNUSED(arg);
  pinMode(LED_PIN, OUTPUT);

  while (1) {
    // Turn LED on.
    digitalWrite(LED_PIN, HIGH);

    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

    xSemaphoreGive(sem);

    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
  }
}

void setup() {
  portBASE_TYPE s1, s2;

  Serial.begin(9600);

  sem = xSemaphoreCreateCounting(1, 0);

  s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
    Serial.println(F("Creation problem"));
    while(1);
  }

  vTaskStartScheduler();
}

void loop() 
{
}
