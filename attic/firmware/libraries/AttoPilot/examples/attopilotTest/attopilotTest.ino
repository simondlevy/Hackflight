#include <AttoPilot.h>

void setup() {
  
  Serial.begin(9600);

}

void loop() {

    float voltage = readVoltage(0);      
    float current = readCurrent(1);      
    
    Serial.print("V = ");
    Serial.print(voltage);
    Serial.print("    A = ");
    Serial.println(current);

}
