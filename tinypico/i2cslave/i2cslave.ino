#include <Wire.h>

static const uint8_t I2C_DEV_ADDR = 0x55;

uint32_t i = 0;

static void onRequest() 
{
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
  Serial.println();
}

void onReceive(int len) 
{
  Serial.printf("onReceive[%d]: ", len);
  while (Wire.available()) {
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() 
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin(I2C_DEV_ADDR);
}

void loop() 
{
  
}
