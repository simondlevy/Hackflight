static const uint8_t PIN = 25;

void setup(void)
{
    Serial.begin(115200);
}

void loop(void)
{
  Serial.println(analogRead(PIN)); 

  delay(100);
}
