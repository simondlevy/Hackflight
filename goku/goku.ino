/**
 *
 * Copyright 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "bootloader.hpp"

#include <hackflight.hpp>
#include <mixers/crazyflie.hpp>

static Imu imu;

void setup()
{
    static HardwareSerial uart = HardwareSerial(PA3, PA2);

    hackflight.init1(PC14, true, &uart);
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
