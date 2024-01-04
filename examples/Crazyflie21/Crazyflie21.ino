static const uint8_t LED_RED_L_PIN = 8;

void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_RED_L_PIN, OUTPUT);
}

void loop(void)
{
    digitalWrite(LED_RED_L_PIN, HIGH);
    delay(500);
    digitalWrite(LED_RED_L_PIN, LOW);
    delay(500);
}
