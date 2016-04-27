#include <FreeSixIMU.h>

#include <Wire.h>

FreeSixIMU sixDOF = FreeSixIMU();

void setup() { 
    
  Serial.begin(9600);
  Wire.begin();
  
  
  sixDOF.init();
}

void loop() { 
  
  float angles[3];     // yaw pitch roll

  sixDOF.getEuler(angles);
      
  Serial.print(angles[0]);
  Serial.print("\t"); 
  Serial.print(angles[1]);
  Serial.print("\t");
  Serial.println(angles[2]);  
}
