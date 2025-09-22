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

static TwoWire wire = TwoWire(SDA_PIN, SCL_PIN);

static Bmi088Accel accel(spi, ACCEL_CS_PIN);

static Bmi088Gyro gyro(spi, GYRO_CS_PIN);

void setup()
{
    Serial.begin(115200);

    wire.begin();

    delay(100);
}

void loop()
{  
}
