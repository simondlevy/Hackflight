#include <BMI088.h>

static TwoWire wire(PC9, PA8);   // internal

Bmi088Accel accel(wire,0x18);

Bmi088Gyro gyro(wire,0x69);

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
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
}

void loop() 
{
  
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
  
  delay(20);
}
