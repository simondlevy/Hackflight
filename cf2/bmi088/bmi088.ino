/**
 *
 * Copyright (C) 2025 Simon D. Levy
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

#include <Wire.h>   

#include <BMI088.h>

static const uint8_t SDA_PIN = PC9;
static const uint8_t SCL_PIN = PA8;
static const uint8_t GYRO_INT_PIN = PC14;

static const uint8_t ACCEL_ADDR = 0x18;
static const uint8_t GYRO_ADDR = 0x69;

static TwoWire wire = TwoWire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(wire, ACCEL_ADDR);

static Bmi088Gyro gyro(wire, GYRO_ADDR);

static volatile bool gyro_flag;

static void gyro_drdy()
{
  gyro_flag = true;
}

void setup() 
{
  int status;
  
  Serial.begin(115200);
  while(!Serial) {}
  
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ);
  status = accel.pinModeInt1(Bmi088Accel::PIN_MODE_PUSH_PULL,Bmi088Accel::PIN_LEVEL_ACTIVE_HIGH);


  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ);
  status = gyro.pinModeInt3(Bmi088Gyro::PIN_MODE_PUSH_PULL,Bmi088Gyro::PIN_LEVEL_ACTIVE_HIGH);
  status = gyro.mapDrdyInt3(true);

  pinMode(GYRO_INT_PIN, INPUT);
  attachInterrupt(GYRO_INT_PIN, gyro_drdy, RISING);  
}

void loop() 
{
  if (gyro_flag) {

    gyro_flag = false;
    
    accel.readSensor();
    
    gyro.readSensor();
    
    Serial.print(accel.getAccelX_mss());
    Serial.print("\t");
    Serial.print(accel.getAccelY_mss());
    Serial.print("\t");
    Serial.print(accel.getAccelZ_mss());
    Serial.print("\t");
    Serial.print(gyro.getGyroX_rads());
    Serial.print("\t");
    Serial.print(gyro.getGyroY_rads());
    Serial.print("\t");
    Serial.print(gyro.getGyroZ_rads());
    Serial.print("\t");
    Serial.print(accel.getTemperature_C());
    Serial.print("\n");
  }
}
