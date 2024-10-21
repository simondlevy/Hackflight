#include <esp_now.h>
#include <WiFi.h>

static uint8_t ONBOARD_ADDRESS[] = {0xAC, 0x0B, 0xFB, 0x6F, 0x6A, 0xD4};

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    Serial.write(incomingData, len);
}

void setup() 
{
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        //Serial.println("Error initializing ESP-NOW");
        //return;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, ONBOARD_ADDRESS, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        //Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() 
{
    uint8_t bytes[18] = {};

    Serial.readBytes(bytes, 18);

    esp_err_t result = esp_now_send(ONBOARD_ADDRESS, bytes, 18);

    if (result != ESP_OK) {
        // Serial.println("Error sending the data");
    }

    delay(10);
}

