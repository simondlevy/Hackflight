#include <LSM6DSOSensor.h>

// Components
LSM6DSOSensor AccGyr(&Wire);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(0);
  
  Wire.begin();
  
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
}

void loop() 
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);

  Serial.print("| Acc[mg]: ");
  Serial.print(accelerometer[0]);
  Serial.print(" ");
  Serial.print(accelerometer[1]);
  Serial.print(" ");
  Serial.print(accelerometer[2]);
  Serial.print(" | Gyr[mdps]: ");
  Serial.print(gyroscope[0]);
  Serial.print(" ");
  Serial.print(gyroscope[1]);
  Serial.print(" ");
  Serial.print(gyroscope[2]);
  Serial.println(" |");
}
