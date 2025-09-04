#include <TinyPICO.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Define a task handle
TaskHandle_t TaskBlinkRedHandle = NULL;
TaskHandle_t TaskBlinkBlueHandle = NULL;

TinyPICO tp = TinyPICO();

// Task function to blink the LED red
void TaskBlinkRed(void *pvParameters) {
  (void) pvParameters; // Suppress unused parameter warning
  for (;;) {
    tp.DotStar_SetPixelColor(255, 0, 0); // Red
    tp.DotStar_Show();
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
    tp.DotStar_SetPixelColor(0, 0, 0); // Off
    tp.DotStar_Show();
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500ms
  }
}

// Task function to blink the LED blue at a different rate
void TaskBlinkBlue(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    tp.DotStar_SetPixelColor(0, 0, 255); // Blue
    tp.DotStar_Show();
    vTaskDelay(pdMS_TO_TICKS(250)); // Delay for 250ms
    tp.DotStar_SetPixelColor(0, 0, 0); // Off
    tp.DotStar_Show();
    vTaskDelay(pdMS_TO_TICKS(250)); // Delay for 250ms
  }
}

void setup() {
  tp.DotStar_SetPower(true); // Turn on power to the DotStar LED
  tp.DotStar_SetBrightness(50); // Set brightness
  
  // Create the two tasks
  xTaskCreatePinnedToCore(
    TaskBlinkRed,           // Task function
    "Blink Red",            // Task name
    2048,                   // Stack size
    NULL,                   // Parameter
    1,                      // Priority (1=low)
    &TaskBlinkRedHandle,    // Task handle
    0                       // Core ID (0 or 1)
  );

  xTaskCreatePinnedToCore(
    TaskBlinkBlue,          // Task function
    "Blink Blue",           // Task name
    2048,                   // Stack size
    NULL,                   // Parameter
    1,                      // Priority (1=low)
    &TaskBlinkBlueHandle,   // Task handle
    1                       // Core ID (0 or 1)
  );
}

void loop() {
  // Empty, as the FreeRTOS tasks handle everything
}

