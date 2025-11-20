#include <ICM42688.h>

#include "bootloader.hpp"

static const uint8_t LED_PIN = PC14;

static const uint8_t SCLK_PIN = PA5;
static const uint8_t MISO_PIN = PA6;
static const uint8_t MOSI_PIN = PA7;

static SPIClass spi;

static ICM42688 IMU(spi, PB12);

static void blinkLed()
{
    digitalWrite(LED_PIN, HIGH);  
    delay(1000);            
    digitalWrite(LED_PIN, LOW); 
    delay(1000);   
}

void setup()
{
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);

    spi.setSCLK(SCLK_PIN);
    spi.setMISO(MISO_PIN);
    spi.setMOSI(MOSI_PIN);

	int status = IMU.begin();

	while (status < 0) {
		Serial.println("IMU initialization unsuccessful");
        delay(500);
	}
}

void loop()
{  
    //blinkLed();

	IMU.getAGT();
	
	Serial.print(IMU.accX(), 6);
	Serial.print("\t");
	Serial.print(IMU.accY(), 6);
	Serial.print("\t");
	Serial.print(IMU.accZ(), 6);
	Serial.print("\t");
	Serial.print(IMU.gyrX(), 6);
	Serial.print("\t");
	Serial.print(IMU.gyrY(), 6);
	Serial.print("\t");
	Serial.print(IMU.gyrZ(), 6);
	Serial.print("\t");
	Serial.println(IMU.temp(), 6);

	delay(100);

    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}
