void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200);
}

void loop()
{
    while (Serial1.available()) {
        printf("0x%02x\n", Serial1.read());
    }
}
