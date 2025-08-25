HardwareSerial Serial2(USART2);

void setup()
{
    Serial.begin(115200);

    Serial2.begin(115200);
}

void loop()
{
    while (Serial2.available()) {
        Serial.print((char)Serial2.read());
    }

    /*
    static uint8_t k;
    Serial2.println((char)('a' + k));
    k = (k + 1) % 26;*/
    delay(100);
}
