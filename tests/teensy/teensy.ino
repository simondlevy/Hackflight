void setup()
{
    Serial.begin(115200);

    Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    while (Serial1.available()) {

        Serial.println(Serial1.read());
    }
}
