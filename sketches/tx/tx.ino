
void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
  Serial.printf("lo:0,hi:4096,c1:%04d,c2:%04d\n", analogRead(4), analogRead(14));

  delay(10);
}
