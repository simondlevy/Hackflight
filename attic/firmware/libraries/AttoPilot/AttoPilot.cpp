#include <AttoPilot.h>
#include <Arduino.h>

float readVoltage(int pin) {
  
  return analogRead(pin) / 19.8;
}

float readCurrent(int pin) {
  
  return analogRead(pin) / 12.75;
}
