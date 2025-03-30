#include "ICM42688.h"

ICM42688 IMU(SPI, PC14);

void setup() {
	// serial to display data
	Serial.begin(115'200);
	while (!Serial) {}

	// start communication with IMU
	int status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		while (1) {}
	}
	Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

void loop() 
{
	// read the sensor
	IMU.getAGT();
	// display the data
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
}
