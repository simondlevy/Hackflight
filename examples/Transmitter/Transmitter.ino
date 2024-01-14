// https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
static uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x84, 0xD5, 0x0C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

// Create a struct_message called myData
static struct_message myData;

static esp_now_peer_info_t peerInfo;

// callback when data is sent
static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void serialEvent(void)
{
    while (Serial.available()) {
        const uint8_t c = Serial.read();
        esp_now_send(broadcastAddress, &c, 1);
    }
}

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

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
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
    // Everything happens in serialEvent()
}
