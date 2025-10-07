void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    while (Serial1.available()) {

        printf("0x%02X\n", Serial1.read());
    }
}
