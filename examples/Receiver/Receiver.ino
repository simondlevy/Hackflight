// https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

#include <esp_now.h>
#include <WiFi.h>

#include <msp.hpp>

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    static Msp msp;

    auto msgtype = msp.parse(incomingData[0]);

    static uint32_t count;

    if (msgtype) {
        Serial.printf("%04d %04d %04d %04d %04d\n",
                msp.parseShort(0),
                msp.parseShort(1),
                msp.parseShort(2),
                msp.parseShort(3),
                msp.parseShort(4));
    }
}

static void error(const char * message)
{
    while (true) {
        Serial.println(message);
        delay(500);
    }
}

void setup() 
{
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        error("Error initializing ESP-NOW");
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() 
{
}
