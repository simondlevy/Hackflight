static HardwareSerial serial2(PA3, PA2);

void setup() 
{
    Serial.begin(115200);

    serial2.begin(115200);
}

void loop()
{
    while (serial2.available()) {
        Serial.printf("%d\n", serial2.read());
    }
}
