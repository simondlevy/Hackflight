void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    while (Serial1.available()) {

        printf("%d\n", Serial1.read());
    }
}
