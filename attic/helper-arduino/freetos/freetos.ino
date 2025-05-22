#include <STM32FreeRTOS.h>

static const uint8_t LED_RED_L = PC0;
static const uint8_t LED_GREEN_L = PC1;
static const uint8_t LED_GREEN_R = PC2;
static const uint8_t LED_RED_R = PC3;
static const uint8_t LED_BLUE_L = PD2;

static const uint8_t LED_PIN = LED_GREEN_R;

static SemaphoreHandle_t sem;

static void Thread1(void* arg) 
{
    (void)arg;

    while (true) {

        // Wait for signal from thread 2.
        xSemaphoreTake(sem, portMAX_DELAY);

        // Turn LED off.
        digitalWrite(LED_PIN, LOW);
    }
}


static void Thread2(void* arg) 
{

    (void)arg;

    pinMode(LED_PIN, OUTPUT);

    while (true) {

        digitalWrite(LED_PIN, HIGH);

        // Sleep for 200 milliseconds.
        vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

        // Signal thread 1 to turn LED off.
        xSemaphoreGive(sem);

        // Sleep for 200 milliseconds.
        vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

        Serial.println(millis());
    }
}
//------------------------------------------------------------------------------
void setup() 
{
    Serial.begin(115200);

    // initialize semaphore
    sem = xSemaphoreCreateCounting(1, 0);

    // create task at priority two
    portBASE_TYPE s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // create task at priority one
    portBASE_TYPE s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // check for creation errors
    if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
        Serial.println(F("Creation problem"));
        while(1);
    }

    // start scheduler
    vTaskStartScheduler();
    Serial.println("Insufficient RAM");
    
    while (true) {
    }
}

void loop() 
{
}
