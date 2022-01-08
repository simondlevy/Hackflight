/*
   Adapted from https://randomnerdtutorials.com/esp-now-two-way-communication-esp32/
 */

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of your sender 
uint8_t senderAddress[] = {0x98, 0xCD, 0xAC, 0xD3, 0x42, 0x3C};

/*
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}*/

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  Serial.println(*incomingData);
}
 
void setup()
{

  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, senderAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop()
{
    /*
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(senderAddress, (uint8_t *)
          &BME280Readings, sizeof(BME280Readings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  
  delay(1000);*/
}
