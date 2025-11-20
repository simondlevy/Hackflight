#include <ICM42688.h>

#include "bootloader.hpp"

#include <imu.hpp>

/*
static const uint8_t LED_PIN = PC14;

static const uint8_t SCLK_PIN = PA5;
static const uint8_t MISO_PIN = PA6;
static const uint8_t MOSI_PIN = PA7;

static SPIClass spi = SPIClass(MOSI_PIN, MISO_PIN, SCLK_PIN);

static ICM42688 IMU(spi, PB12);
*/

static Imu imu;

/*
static void blinkLed()
{
    digitalWrite(LED_PIN, HIGH);  
    delay(1000);            
    digitalWrite(LED_PIN, LOW); 
    delay(1000);   
}*/

void setup()
{
    Serial.begin(115200);

    imu.init();

    /*
    pinMode(LED_PIN, OUTPUT);

	int status = IMU.begin();

	while (status < 0) {
		Serial.println("IMU initialization unsuccessful");
        delay(500);
	}*/
}

void loop()
{  
    /*
    //blinkLed();

	IMU.getAGT();
	
Serial.print("\t");
    Serial.println(IMU.temp(), 6);
     */


    int16_t gx=0, gy=0, gz=0, ax=0, ay=0, az=0;
    imu.device_read(gx, gy, gz, ax, ay, az);

    Serial.print("ax=");
	Serial.print(ax);
	Serial.print("\tay=");
	Serial.print(ay);
	Serial.print("\taz=");
	Serial.print(az);
	Serial.print("\tgx=");
	Serial.print(gx);
	Serial.print("\tgy=");
    Serial.print(gy);
    Serial.print("\tgz=");
    Serial.println(gz);
    
    delay(10);

    if (Serial.available() && Serial.read() == 'R') {
        Bootloader::jump();
    }
}
