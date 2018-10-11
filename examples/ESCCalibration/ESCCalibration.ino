/*
   ESCCalibration.ino : Hackflight sketch for calibrating Electronic Speed Controllers

   Hardware support:

       https://github.com/simondlevy/grumpyoldpizza

   Copyright (c) 2018 Juan Gallostra Acin

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Hackflight is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with Hackflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// PWM Values
static uint16_t BASELINE = 1000;
static uint16_t MIDVAL   = 1500;
static uint16_t MAXVAL   = 2000;

static uint8_t MOTOR_PINS[4] = {3, 4, 5, 6};
static uint8_t LED_R = 13;
static uint8_t LED_G = 38;
static uint8_t LED_B = 26;

static void initLedSequence()
{
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
  
  digitalWrite(LED_R, LOW);
  delay(500);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, LOW);
  delay(500);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, LOW);
  delay(500);
}

void setup(void)
{

  pinMode(LED_R, OUTPUT);  
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  initLedSequence();
  
  // Blue LED ON when calibrating (first part)
  for (int k=0; k<4; ++k)
  {
    pinMode(MOTOR_PINS[k], OUTPUT);    
    analogWrite(MOTOR_PINS[k], MAXVAL >> 3);
  }
  delay(10000);
  digitalWrite(LED_B, HIGH);
  
  // Green LED ON when calibrating (second part)
  digitalWrite(LED_G, LOW);
  for (int k=0; k<4; ++k)
  {
    analogWrite(MOTOR_PINS[k], BASELINE >> 3);
  }
  delay(10000);
  digitalWrite(LED_G, HIGH);
}

void loop(void)
{

}


