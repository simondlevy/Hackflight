#include "bootloader.hpp"

#include <imu.hpp>

static Imu imu;

void setup()
{
    Serial.begin(115200);

    imu.init();
}

void loop()
{  
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
