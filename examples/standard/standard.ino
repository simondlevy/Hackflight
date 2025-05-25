#include <arduino_freertos.h>

FLASHMEM __attribute__((noinline)) void setup()
{
    Serial.begin(0);
    delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }
}

void loop()
{
}
