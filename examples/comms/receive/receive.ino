HardwareSerial Serial2(USART2);

void setup() 
{
    Serial2.begin(115200);
}

void loop() 
{
    static uint8_t k;

    Serial2.write(65 + k);

    k = (k + 1) % 26;

    delay(50);
}
