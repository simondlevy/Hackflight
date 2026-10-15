#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 22

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

void read_dist() {
  Serial.print("Distance = "); Serial.println(vl53.distance());
  vl53.clearInterrupt();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X interrupt demo"));

  Wire.begin();
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Interrupt mode (RISING/FALLING) must match polarity (1/0)
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  
  // 1 = normally low, assert HIGH (RISING) <-- default
  vl53.VL53L1X_SetInterruptPolarity(1);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), read_dist, RISING);
  
  // 0 = normally high, assert low (FALLING)
  //vl53.VL53L1X_SetInterruptPolarity(0);
  //attachInterrupt(digitalPinToInterrupt(IRQ_PIN), read_dist, FALLING);

  // Clear interrupt to get things started
  vl53.clearInterrupt();
}

void loop() {
  // nothing to do, everything happens in ISR
} 
