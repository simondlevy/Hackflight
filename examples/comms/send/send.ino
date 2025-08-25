HardwareSerial Serial2(USART2);

void setup() 
{
    Serial.begin(115200);

    Serial2.begin(115200);
}

void loop() 
{
    if (Serial2.available()) {
        Serial.print((char)Serial2.read());
    }
}
