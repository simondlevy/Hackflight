#include <esp_now.h>
#include <WiFi.h>

static uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4};

static esp_now_peer_info_t peerInfo;

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  Serial.print("Bytes received: ");
  Serial.println(len);
}
 
void setup() 
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() 
{
  const uint8_t bytes[20] =
    {0x24,0x4D,0x3C,0x0C,0xC8,0x00,0x00,0x00,0x04,0x00,0x04,0xFA,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x3D};

  esp_err_t result = esp_now_send(broadcastAddress, bytes, 20);
   
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }

  delay(100);
}

