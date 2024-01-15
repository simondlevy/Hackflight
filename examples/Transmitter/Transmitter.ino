// https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
static uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x84, 0xD5, 0x0C};

static void error(const char * message)
{
    while (true) {
        Serial.println(message);
        delay(500);
    }
}

void setup(void) 
{
    // We will read input from USB port, via serialEvent() callback
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        error("Error initializing ESP-NOW");
    }

    // Register peer
    static esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        error("Failed to add peer");
    }
}

void loop(void) 
{
    const auto avail = Serial.available();

    static uint8_t buf[256];

    Serial.readBytes(buf, avail);

    esp_now_send(broadcastAddress, buf, avail);
}
