#include <esp_now.h>
#include <WiFi.h>

static uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x83, 0x9B, 0xA4};

static float temperature;
static float humidity;
static float pressure;

static String success;

typedef struct struct_message 
{
    float temp;
    float hum;
    float pres;
} struct_message;

static struct_message BME280Readings;

static struct_message incomingReadings;

static esp_now_peer_info_t peerInfo;

static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  Serial.print("Bytes received: ");
  Serial.println(len);
}
 
static void getReadings()
{
  temperature = 99;
  humidity = 100;
  pressure = 101;
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
  getReadings();
 
  BME280Readings.temp = temperature;
  BME280Readings.hum = humidity;
  BME280Readings.pres = pressure;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
   
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }

  delay(100);
}

